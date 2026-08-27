/**
 * Persistent synchronous SMP1 -> BUMI3 GMR service.
 *
 * stdin carries framed GMRQ requests and a duplicated copy of the original
 * stdout fd carries binary GMRA responses.  stdout itself is redirected to
 * stderr before constructing GMR so diagnostics can never corrupt responses.
 */

#include "gmr/frame_queue.hpp"
#include "gmr/gmr_mink.hpp"
#include "readers/smplx_reader.hpp"

#include <Eigen/Dense>

#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
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
    bool offset_to_ground = true;
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
        else if (argument == "--fixed-ground-offset")
            cfg.fixed_ground_offset = std::stod(next());
        else if (argument == "--ground-clearance")
            cfg.ground_clearance = std::stod(next());
        else if (argument == "--offset-to-ground") cfg.offset_to_ground = true;
        else if (argument == "--no-offset-to-ground") cfg.offset_to_ground = false;
        else if (argument == "--help" || argument == "-h") {
            std::cerr << "Usage: " << argv[0]
                      << " [--xml PATH] [--ik-config PATH]"
                      << " [--ground-clearance M] [--no-offset-to-ground]\n";
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
    return cfg;
}

std::unique_ptr<gmr_mink::GMR> makeGmr(const Config& cfg) {
    auto result = std::make_unique<gmr_mink::GMR>(
        cfg.xml, cfg.ik, cfg.human_height, cfg.damping, false);
    result->setGroundOffset(cfg.fixed_ground_offset);
    result->setGroundClearance(cfg.ground_clearance);
    return result;
}

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
        auto gmr = makeGmr(cfg);
        std::cerr << "[GMR batch] ready xml=" << cfg.xml
                  << " ik=" << cfg.ik << " qpos=28\n";

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
                    for (uint32_t index = 0; index < iterations; ++index)
                        qpos = gmr->retarget(raw.body_data, cfg.offset_to_ground);
                } else if (request.operation == kOpFrame) {
                    qpos = gmr->retarget(raw.body_data, cfg.offset_to_ground);
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
