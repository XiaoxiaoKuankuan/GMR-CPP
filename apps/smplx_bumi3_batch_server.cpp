/**
 * Persistent synchronous SMP1 -> BUMI3 GMR service.
 *
 * stdin carries framed GMRQ requests and a duplicated copy of the original
 * stdout fd carries binary GMRA responses.  stdout itself is redirected to
 * stderr before constructing GMR so diagnostics can never corrupt responses.
 */

#include "gmr/frame_queue.hpp"
#include "gmr/foot_contact_json.hpp"
#include "gmr/geometry_ground.hpp"
#include "gmr/gmr_mink.hpp"
#include "readers/smplx_reader.hpp"

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

namespace {

constexpr std::array<char, 4> kRequestMagic{'G', 'M', 'R', 'Q'};
constexpr std::array<char, 4> kResponseMagic{'G', 'M', 'R', 'A'};
constexpr uint16_t kVersion = 1;
constexpr uint16_t kOpFrame = 1;
constexpr uint16_t kOpReset = 2;
constexpr uint16_t kOpQuit = 3;
constexpr uint16_t kStatusOk = 0;
constexpr uint16_t kStatusError = 1;
constexpr size_t kSmp1Bytes = 412;
constexpr uint32_t kBumiQposDim = 28;
constexpr size_t kRequestHeaderBytes = 24;
constexpr size_t kResponseHeaderBytes = 32;

uint16_t readU16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) |
           static_cast<uint16_t>(p[1]) << 8;
}

uint32_t readU32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           static_cast<uint32_t>(p[1]) << 8 |
           static_cast<uint32_t>(p[2]) << 16 |
           static_cast<uint32_t>(p[3]) << 24;
}

void writeU16(uint8_t* p, uint16_t value) {
    p[0] = static_cast<uint8_t>(value);
    p[1] = static_cast<uint8_t>(value >> 8);
}

void writeU32(uint8_t* p, uint32_t value) {
    for (int i = 0; i < 4; ++i)
        p[i] = static_cast<uint8_t>(value >> (8 * i));
}

void writeU64(uint8_t* p, uint64_t value) {
    for (int i = 0; i < 8; ++i)
        p[i] = static_cast<uint8_t>(value >> (8 * i));
}

uint32_t crc32(const uint8_t* data, size_t size) {
    uint32_t value = 0xFFFF'FFFFU;
    for (size_t index = 0; index < size; ++index) {
        value ^= data[index];
        for (int bit = 0; bit < 8; ++bit)
            value = (value >> 1) ^
                    (0xEDB8'8320U & static_cast<uint32_t>(-(value & 1U)));
    }
    return value ^ 0xFFFF'FFFFU;
}

bool readExact(int fd, void* destination, size_t size) {
    auto* output = static_cast<uint8_t*>(destination);
    size_t done = 0;
    while (done < size) {
        const ssize_t count = ::read(fd, output + done, size - done);
        if (count == 0) return false;
        if (count < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error("stdin read failed");
        }
        done += static_cast<size_t>(count);
    }
    return true;
}

void writeExact(int fd, const void* source, size_t size) {
    const auto* input = static_cast<const uint8_t*>(source);
    size_t done = 0;
    while (done < size) {
        const ssize_t count = ::write(fd, input + done, size - done);
        if (count < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error("response write failed");
        }
        done += static_cast<size_t>(count);
    }
}

struct Request {
    uint16_t operation = 0;
    uint32_t sequence = 0;
    uint32_t reset_iterations = 0;
    std::vector<uint8_t> payload;
};

bool readRequest(Request& result) {
    std::array<uint8_t, kRequestHeaderBytes> header{};
    if (!readExact(STDIN_FILENO, header.data(), header.size())) return false;
    if (std::memcmp(header.data(), kRequestMagic.data(), 4) != 0 ||
        readU16(header.data() + 4) != kVersion)
        throw std::runtime_error("unsupported GMRQ magic/version");
    result.operation = readU16(header.data() + 6);
    result.sequence = readU32(header.data() + 8);
    const uint32_t payload_size = readU32(header.data() + 12);
    result.reset_iterations = readU32(header.data() + 16);
    const uint32_t expected_crc = readU32(header.data() + 20);
    if (payload_size > 1024 * 1024)
        throw std::runtime_error("GMRQ payload is unreasonably large");
    result.payload.resize(payload_size);
    if (payload_size && !readExact(STDIN_FILENO, result.payload.data(), payload_size))
        throw std::runtime_error("truncated GMRQ payload");
    if (crc32(result.payload.data(), result.payload.size()) != expected_crc)
        throw std::runtime_error("GMRQ payload CRC32 mismatch");
    return true;
}

void sendResponse(int response_fd,
                  uint16_t status,
                  uint32_t sequence,
                  uint32_t qpos_dim,
                  const std::vector<uint8_t>& payload,
                  uint64_t elapsed_us) {
    std::array<uint8_t, kResponseHeaderBytes> header{};
    std::memcpy(header.data(), kResponseMagic.data(), 4);
    writeU16(header.data() + 4, kVersion);
    writeU16(header.data() + 6, status);
    writeU32(header.data() + 8, sequence);
    writeU32(header.data() + 12, qpos_dim);
    writeU32(header.data() + 16, static_cast<uint32_t>(payload.size()));
    writeU32(header.data() + 20, crc32(payload.data(), payload.size()));
    writeU64(header.data() + 24, elapsed_us);
    writeExact(response_fd, header.data(), header.size());
    if (!payload.empty()) writeExact(response_fd, payload.data(), payload.size());
}

std::vector<uint8_t> qposPayload(const Eigen::VectorXd& qpos) {
    if (qpos.size() != kBumiQposDim)
        throw std::runtime_error("BUMI3 GMR qpos dimension is not 28");
    std::vector<uint8_t> payload(kBumiQposDim * sizeof(float));
    for (uint32_t index = 0; index < kBumiQposDim; ++index) {
        const float value = static_cast<float>(qpos[static_cast<int>(index)]);
        if (!std::isfinite(value))
            throw std::runtime_error("GMR returned a non-finite qpos");
        static_assert(sizeof(float) == 4);
        std::memcpy(payload.data() + index * 4, &value, 4);
    }
    return payload;
}

struct Config {
    std::string xml;
    std::string ik;
    double human_height = 1.8;
    double damping = 1.0;
    double fixed_ground_offset = 0.0;
    double ground_clearance = 0.02;
    double source_ground_clearance = 0.0;
    bool offset_to_ground = true;
    bool fixed_ground_offset_explicit = false;
    int foot_contact_override = -1;
    double foot_contact_weight_scale = 1.0;
};

Config parseArgs(int argc, char** argv) {
    namespace fs = std::filesystem;
    Config cfg;
    const fs::path executable = fs::canonical(argv[0]);
    const fs::path root = executable.parent_path().parent_path();
    cfg.xml = (root / "assets/bumi3/mjcf/bumi3.xml").string();
    cfg.ik = (root / "config/ik_configs/smplx_to_bumi3_auto.json").string();
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        auto next = [&]() -> std::string {
            if (++index >= argc)
                throw std::runtime_error("missing value for " + argument);
            return argv[index];
        };
        if (argument == "--xml") cfg.xml = next();
        else if (argument == "--ik-config") cfg.ik = next();
        else if (argument == "--human-height") cfg.human_height = std::stod(next());
        else if (argument == "--damping") cfg.damping = std::stod(next());
        else if (argument == "--fixed-ground-offset") {
            cfg.fixed_ground_offset = std::stod(next());
            cfg.fixed_ground_offset_explicit = true;
        }
        else if (argument == "--ground-clearance")
            cfg.ground_clearance = std::stod(next());
        else if (argument == "--source-ground-clearance")
            cfg.source_ground_clearance = std::stod(next());
        else if (argument == "--offset-to-ground") cfg.offset_to_ground = true;
        else if (argument == "--no-offset-to-ground") cfg.offset_to_ground = false;
        else if (argument == "--foot-contact-constraints")
            cfg.foot_contact_override = 1;
        else if (argument == "--no-foot-contact-constraints")
            cfg.foot_contact_override = 0;
        else if (argument == "--foot-contact-weight-scale")
            cfg.foot_contact_weight_scale = std::stod(next());
        else if (argument == "--help" || argument == "-h") {
            std::cerr << "Usage: " << argv[0]
                      << " [--xml PATH] [--ik-config PATH]"
                      << " [--ground-clearance M]"
                      << " [--source-ground-clearance M]"
                      << " [--no-offset-to-ground]"
                      << " [--no-foot-contact-constraints]"
                      << " [--foot-contact-weight-scale SCALE]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + argument);
        }
    }
    if (!fs::is_regular_file(cfg.xml))
        throw std::runtime_error("robot XML not found: " + cfg.xml);
    if (!fs::is_regular_file(cfg.ik))
        throw std::runtime_error("IK config not found: " + cfg.ik);
    if (!std::isfinite(cfg.ground_clearance) || cfg.ground_clearance < 0.0)
        throw std::runtime_error("ground clearance must be finite and >= 0");
    if (!std::isfinite(cfg.source_ground_clearance) ||
        cfg.source_ground_clearance < 0.0)
        throw std::runtime_error(
            "source ground clearance must be finite and >= 0");
    if (!std::isfinite(cfg.fixed_ground_offset))
        throw std::runtime_error("fixed ground offset must be finite");
    if (!std::isfinite(cfg.foot_contact_weight_scale) ||
        cfg.foot_contact_weight_scale < 0.0 ||
        cfg.foot_contact_weight_scale > 5.0)
        throw std::runtime_error(
            "foot contact weight scale must be finite and in [0, 5]");
    return cfg;
}

std::unique_ptr<gmr_mink::GMR> makeGmr(const Config& cfg) {
    auto result = std::make_unique<gmr_mink::GMR>(
        cfg.xml, cfg.ik, cfg.human_height, cfg.damping, false);
    result->setGroundOffset(cfg.fixed_ground_offset);
    result->setGroundClearance(cfg.ground_clearance);
    return result;
}

nlohmann::json readJson(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open " + path);
    nlohmann::json value;
    input >> value;
    return value;
}

gmr::FootObservation relativeToSourceGround(
    gmr::FootObservation observation, double source_ground_z) {
    observation.center_world.z() -= source_ground_z;
    for (auto& point : observation.points_world)
        point.z() -= source_ground_z;
    return observation;
}

class BatchFootContactRuntime {
public:
    BatchFootContactRuntime(const Config& cfg, const nlohmann::json& ik_profile)
        : cfg_(cfg), constraint_config_(
              gmr::footConstraintConfigFromJson(ik_profile)) {
        enabled_ = cfg.foot_contact_override >= 0
            ? cfg.foot_contact_override != 0
            : constraint_config_.enabled;
        if (!enabled_) return;
        if (!constraint_config_.enabled || !ik_profile.contains("foot_contact"))
            throw std::runtime_error(
                "foot contact was requested but the IK config has no enabled profile");
        const auto& foot = ik_profile.at("foot_contact");
        source_left_ = gmr::footSoleDefinitionFromJson(
            foot.at("source").at("left"), "foot_contact.source.left");
        source_right_ = gmr::footSoleDefinitionFromJson(
            foot.at("source").at("right"), "foot_contact.source.right");
        detector_ = std::make_unique<gmr::FootContactDetector>(
            gmr::footDetectorConfigFromJson(foot));
        ground_tracker_ = std::make_unique<gmr::SourceGroundTracker>(
            gmr::sourceGroundTrackerConfigFromJson(foot));
        target_ground_ = std::make_unique<gmr::TargetGroundAligner>(
            cfg.xml, std::vector<std::string>{
                constraint_config_.left.body_name,
                constraint_config_.right.body_name});
    }

    bool enabled() const { return enabled_; }

    void configureSolver(gmr_mink::GMR& solver) const {
        solver.setFootContactEnabled(enabled_);
        if (enabled_)
            solver.setFootContactWeightScale(cfg_.foot_contact_weight_scale);
    }

    Eigen::VectorXd reset(gmr_mink::GMR& solver, const gmr::RawFrame& raw,
                          uint32_t iterations) {
        configureSolver(solver);
        initialized_ = false;
        previous_timestamp_ = std::numeric_limits<double>::quiet_NaN();
        previous_frame_number_ = 0;
        have_previous_frame_number_ = false;
        current_frame_dt_ = 1.0 / 30.0;
        have_output_qpos_ = false;
        output_root_limit_count_ = 0;
        latest_state_ = {};
        if (ground_tracker_) ground_tracker_->reset();
        ground_reacquiring_ = false;
        if (!enabled_) {
            Eigen::VectorXd qpos;
            for (uint32_t index = 0; index < iterations; ++index)
                qpos = solver.retarget(raw.body_data, cfg_.offset_to_ground);
            return qpos;
        }

        detector_->reset();
        latest_state_ = detect(raw, true);
        const gmr::BodyMap body_data = correctedBodyData(raw.body_data);
        if (!cfg_.fixed_ground_offset_explicit) {
            const double ground_offset = solver.calibrateGroundOffset(
                body_data, cfg_.source_ground_clearance);
            std::cerr << "[GMR batch contact] fixed source ground offset="
                      << ground_offset << "m clearance="
                      << cfg_.source_ground_clearance << "m\n";
        }

        // First converge the unconstrained whole-body IK, then seat the real
        // robot foot mesh and initialize world-space stance anchors.  Contact
        // constraints are warmed only after a geometrically valid seed exists.
        Eigen::VectorXd qpos;
        for (uint32_t index = 0; index < iterations; ++index)
            qpos = solver.retarget(body_data, false);
        target_ground_->align(qpos, constraint_config_.ground_z);
        solver.setConfiguration(qpos);
        solver.initializeFootContacts(latest_state_);
        solver.settleFootContacts();
        const int contact_warmup = std::max(
            10, constraint_config_.settings.transition_frames);
        for (int index = 0; index < contact_warmup; ++index)
            qpos = solver.retarget(body_data, false);
        initialized_ = true;
        reportDiagnostics(solver, "reset");
        return finalizeOutput(solver, std::move(qpos));
    }

    Eigen::VectorXd frame(gmr_mink::GMR& solver, const gmr::RawFrame& raw) {
        if (!enabled_)
            return solver.retarget(raw.body_data, cfg_.offset_to_ground);
        if (!initialized_)
            return reset(solver, raw, 1000);
        const gmr::FootContactState previous_state = latest_state_;
        latest_state_ = detect(raw, false);
        const gmr::BodyMap body_data = correctedBodyData(raw.body_data);
        const bool contact_entered =
            (latest_state_.left_stance && !previous_state.left_stance) ||
            (latest_state_.right_stance && !previous_state.right_stance);
        const bool anchor_activated =
            (latest_state_.left_stance && previous_state.left_stance &&
             previous_state.left_forced && !latest_state_.left_forced) ||
            (latest_state_.right_stance && previous_state.right_stance &&
             previous_state.right_forced && !latest_state_.right_forced);
        if (contact_entered || anchor_activated) {
            // Release liftoff immediately, but keep newly entering feet free
            // while the current human frame is followed once.  Only then
            // capture the current-frame world anchor and enable the gradual
            // hard-plane transition.  This avoids pulling a landing back to
            // a stale previous-frame anchor.
            gmr::FootContactState pre_contact = latest_state_;
            if (latest_state_.left_stance && !previous_state.left_stance) {
                pre_contact.left_stance = false;
                pre_contact.left_forced = false;
            } else if (latest_state_.left_stance && previous_state.left_stance &&
                       previous_state.left_forced &&
                       !latest_state_.left_forced) {
                pre_contact.left_forced = true;
            }
            if (latest_state_.right_stance && !previous_state.right_stance) {
                pre_contact.right_stance = false;
                pre_contact.right_forced = false;
            } else if (latest_state_.right_stance &&
                       previous_state.right_stance &&
                       previous_state.right_forced &&
                       !latest_state_.right_forced) {
                pre_contact.right_forced = true;
            }
            solver.setFootContactState(pre_contact, false);
            Eigen::VectorXd qpos = solver.retarget(body_data, false);
            // Activate the new anchor at the already-followed current pose,
            // but defer its first constrained solve to the next input frame.
            // One SMP1 frame therefore advances the IK timeline exactly once.
            solver.setFootContactState(latest_state_, false);
            return finalizeOutput(solver, std::move(qpos));
        }
        solver.setFootContactState(latest_state_);
        return finalizeOutput(
            solver, solver.retarget(body_data, false));
    }

private:
    gmr::FootContactState detect(const gmr::RawFrame& raw, bool seed) {
        auto left = gmr::observeFoot(
            raw.body_data, gmr::FootSide::Left, source_left_);
        auto right = gmr::observeFoot(
            raw.body_data, gmr::FootSide::Right, source_right_);
        double timestamp = raw.stamp_ns > 0
            ? static_cast<double>(raw.stamp_ns) * 1e-9 : 0.0;
        if (!seed && std::isfinite(previous_timestamp_) &&
            timestamp <= previous_timestamp_) {
            const bool duplicate_packet = have_previous_frame_number_ &&
                raw.frame_number == previous_frame_number_;
            if (!duplicate_packet)
                timestamp = previous_timestamp_ + 0.02;
        }
        if (!seed && std::isfinite(previous_timestamp_) &&
            timestamp > previous_timestamp_ &&
            timestamp - previous_timestamp_ <= 0.2)
            current_frame_dt_ = timestamp - previous_timestamp_;

        const bool had_ground = ground_tracker_->initialized();
        ground_tracker_->update(left, right, latest_state_, timestamp);
        if (!had_ground) {
            std::cerr << "[GMR batch contact] locked SMP1 source ground z="
                      << ground_tracker_->initialGroundZ() << "m\n";
        }
        if (ground_tracker_->reacquiring() != ground_reacquiring_) {
            ground_reacquiring_ = ground_tracker_->reacquiring();
            std::cerr << "[GMR batch contact] source ground reacquire="
                      << (ground_reacquiring_ ? "on" : "off")
                      << " correction=" << ground_tracker_->correction()
                      << "m flight_frames="
                      << ground_tracker_->flightFrames() << "\n";
        }
        left = relativeToSourceGround(
            std::move(left), ground_tracker_->groundZ());
        right = relativeToSourceGround(
            std::move(right), ground_tracker_->groundZ());

        gmr::FootContactState state;
        const int updates = seed
            ? std::max(1, detector_->config().enter_frames) : 1;
        for (int index = 0; index < updates; ++index)
            state = detector_->update(left, right, timestamp);
        previous_timestamp_ = timestamp;
        previous_frame_number_ = raw.frame_number;
        have_previous_frame_number_ = true;
        return state;
    }

    gmr::BodyMap correctedBodyData(const gmr::BodyMap& body_data) const {
        gmr::BodyMap corrected = body_data;
        const double correction = ground_tracker_->correction();
        for (auto& [name, body] : corrected) {
            (void)name;
            body.position.z() -= correction;
        }
        return corrected;
    }

    Eigen::VectorXd finalizeOutput(gmr_mink::GMR& solver,
                                   Eigen::VectorXd qpos) {
        if (!have_output_qpos_) {
            previous_output_qpos_ = qpos;
            have_output_qpos_ = true;
            return qpos;
        }
        const double dt = std::clamp(
            current_frame_dt_, 1.0 / 240.0, 0.1);
        const double maximum =
            constraint_config_.settings.max_output_root_horizontal_velocity * dt;
        Eigen::Vector2d delta = qpos.head<2>() -
                                previous_output_qpos_.head<2>();
        if (delta.norm() > maximum) {
            delta *= maximum / delta.norm();
            qpos.head<2>() = previous_output_qpos_.head<2>() + delta;
            solver.setConfiguration(qpos);
            ++output_root_limit_count_;
            if (output_root_limit_count_ == 1)
                std::cerr
                    << "[GMR batch contact] causal output root limit active: "
                    << constraint_config_.settings
                           .max_output_root_horizontal_velocity
                    << "m/s horizontal\n";
        }
        previous_output_qpos_ = qpos;
        return qpos;
    }

    static void reportDiagnostics(const gmr_mink::GMR& solver,
                                  const char* stage) {
        const auto& status = solver.footContactDiagnostics();
        std::cerr << "[GMR batch contact] " << stage
                  << " stance=(" << status.left_stance << ','
                  << status.right_stance << ") tilt_deg=("
                  << status.left_tilt_degrees << ','
                  << status.right_tilt_degrees << ") heel_toe_m=("
                  << status.left_heel_toe_height_difference << ','
                  << status.right_heel_toe_height_difference << ")\n";
    }

    Config cfg_;
    gmr::FootConstraintConfig constraint_config_;
    bool enabled_ = false;
    bool initialized_ = false;
    double previous_timestamp_ = std::numeric_limits<double>::quiet_NaN();
    uint32_t previous_frame_number_ = 0;
    bool have_previous_frame_number_ = false;
    double current_frame_dt_ = 1.0 / 30.0;
    gmr::FootSoleDefinition source_left_;
    gmr::FootSoleDefinition source_right_;
    gmr::FootContactState latest_state_;
    std::unique_ptr<gmr::FootContactDetector> detector_;
    std::unique_ptr<gmr::SourceGroundTracker> ground_tracker_;
    std::unique_ptr<gmr::TargetGroundAligner> target_ground_;
    bool ground_reacquiring_ = false;
    bool have_output_qpos_ = false;
    Eigen::VectorXd previous_output_qpos_;
    uint64_t output_root_limit_count_ = 0;
};

gmr::RawFrame decodeSmp1(const std::vector<uint8_t>& payload) {
    if (payload.size() != kSmp1Bytes)
        throw std::runtime_error("FRAME/RESET payload must contain one 412-byte SMP1 packet");
    gmr::RawFrame raw;
    if (!gmr::SmplxReader::decodePacket(payload.data(), payload.size(), raw))
        throw std::runtime_error("invalid SMP1 packet");
    return raw;
}

}  // namespace

int main(int argc, char** argv) {
    int response_fd = -1;
    try {
        const Config cfg = parseArgs(argc, argv);
        response_fd = ::dup(STDOUT_FILENO);
        if (response_fd < 0 || ::dup2(STDERR_FILENO, STDOUT_FILENO) < 0)
            throw std::runtime_error("failed to isolate binary response fd");
        const nlohmann::json ik_profile = readJson(cfg.ik);
        BatchFootContactRuntime foot_contact(cfg, ik_profile);
        auto gmr = makeGmr(cfg);
        foot_contact.configureSolver(*gmr);
        std::cerr << "[GMR batch] ready xml=" << cfg.xml
                  << " ik=" << cfg.ik << " qpos=28 foot_contact="
                  << (foot_contact.enabled() ? "on" : "off") << "\n";
        if (foot_contact.enabled() && cfg.offset_to_ground)
            std::cerr
                << "[GMR batch contact] per-frame offset-to-ground disabled; "
                   "using one fixed source-ground calibration\n";

        while (true) {
            Request request;
            if (!readRequest(request)) break;
            if (request.operation == kOpQuit) break;
            const auto started = std::chrono::steady_clock::now();
            try {
                gmr::RawFrame raw = decodeSmp1(request.payload);
                Eigen::VectorXd qpos;
                if (request.operation == kOpReset) {
                    gmr = makeGmr(cfg);
                    const uint32_t iterations =
                        request.reset_iterations == 0 ? 1000 : request.reset_iterations;
                    qpos = foot_contact.reset(*gmr, raw, iterations);
                } else if (request.operation == kOpFrame) {
                    qpos = foot_contact.frame(*gmr, raw);
                } else {
                    throw std::runtime_error("unsupported GMRQ operation");
                }
                const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now() - started);
                sendResponse(response_fd, kStatusOk, request.sequence,
                             kBumiQposDim, qposPayload(qpos),
                             static_cast<uint64_t>(elapsed.count()));
            } catch (const std::exception& error) {
                const std::string message = error.what();
                const std::vector<uint8_t> payload(message.begin(), message.end());
                sendResponse(response_fd, kStatusError, request.sequence,
                             0, payload, 0);
            }
        }
        if (response_fd >= 0) ::close(response_fd);
        std::cerr << "[GMR batch] stopped\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[GMR batch fatal] " << error.what() << "\n";
        if (response_fd >= 0) ::close(response_fd);
        return 1;
    }
}
