/**
 * gem_server.cpp — GEM-SMPL UDP -> GMR IK -> Redis.
 *
 * This entry point intentionally keeps the existing GMR core untouched.
 * Safety difference from the older servers: it stops refreshing Redis when
 * no fresh GEM packet has arrived for --stale-ms, so the Redis TTL can expire.
 */

#include "gmr/frame_queue.hpp"
#include "gmr/gmr_mink.hpp"
#include "gmr/motion_buffer.hpp"
#include "gmr/mujoco_viewer.hpp"
#include "gmr/redis_publisher.hpp"
#include "readers/gem_reader.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

std::atomic<bool> g_stop{false};
static void sigHandler(int) { g_stop = true; }

namespace {

struct Joystick {
    static constexpr int kButtonA  = 0;
    static constexpr int kButtonR1 = 5;
    static constexpr int kMaxButtons = 32;

    Joystick() {
        fd_ = ::open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
        if (fd_ < 0)
            std::cerr << "[Joystick] /dev/input/js0 not found; "
                         "use --always for simulation/testing\n";
        std::memset(buttons_, 0, sizeof(buttons_));
    }

    ~Joystick() {
        if (fd_ >= 0) ::close(fd_);
    }

    void update() {
        if (fd_ < 0) return;
        js_event event{};
        while (::read(fd_, &event, sizeof(event)) == sizeof(event)) {
            if ((event.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON &&
                event.number < kMaxButtons) {
                buttons_[event.number] = event.value;
            }
        }
    }

    bool pressed(int button) const {
        return fd_ >= 0 && button >= 0 &&
               button < kMaxButtons && buttons_[button] != 0;
    }

private:
    int fd_ = -1;
    int buttons_[kMaxButtons]{};
};

struct Config {
    std::string bind_ip       = "0.0.0.0";
    int         port          = 7001;
    std::string xml_file;
    std::string ik_config;
    std::string preset        = "e1";
    double      human_height  = 1.8;
    double      damping       = 1.0;
    double      publish_hz    = 50.0;
    double      stale_ms      = 250.0;
    bool        vis           = false;
    bool        vis_smpl_targets = false;
    int         viewer_width  = 640;
    int         viewer_height = 480;
    std::string viewer_follow_body = "pelvis";
    bool        offset_to_ground = false;
    bool        require_buttons = true;
    gmr::GemProtocol gem_protocol = gmr::GemProtocol::Any;

    gmr::RedisPublisher::Config redis;
};

void usage(const char* program) {
    std::printf(
        "Usage: %s [options]\n"
        "  --bind <IPv4>             UDP bind address (default 0.0.0.0)\n"
        "  --port <port>             GEM UDP port (default 7001)\n"
        "  --gem-protocol <mode>     auto|gem1|gem2 (default auto)\n"
        "  --xml <path>              MuJoCo robot XML\n"
        "  --ik-config <path>        human-to-E1 IK JSON\n"
        "  --preset <e1>             robot preset (default e1)\n"
        "  --human-height <m>        Height used by GMR scaling (default 1.8)\n"
        "  --damping <value>         IK damping (default 1.0)\n"
        "  --redis-host <host>       default 127.0.0.1\n"
        "  --redis-port <port>       default 6379\n"
        "  --redis-db <db>           default 0\n"
        "  --redis-key <key>         override preset key\n"
        "  --hz <rate>               Redis publish rate (default 50)\n"
        "  --ttl-ms <ms>             Redis current-key TTL (default 200)\n"
        "  --stale-ms <ms>           stop publishing after input loss (default 250)\n"
        "  --lin-vel-alpha <0..1>    root linear-velocity EMA\n"
        "  --ang-vel-alpha <0..1>    root angular-velocity EMA\n"
        "  --lin-vel-max <m/s>       reject linear-velocity spikes\n"
        "  --ang-vel-max <rad/s>     reject angular-velocity spikes\n"
        "  --pelvis-z-offset <m>     output-only pelvis z offset (default 0)\n"
        "  --offset-to-ground        per-frame lowest-foot grounding (off by default)\n"
        "  --no-offset-to-ground     keep sender's initial ground (default)\n"
        "  --always                  bypass A+R1 joystick publish gate\n"
        "  --vis                     open MuJoCo viewer\n"
        "  --vis-smpl-targets        show GEM2 raw/IK/robot target overlay\n"
        "  --viewer-width <px>       default 640\n"
        "  --viewer-height <px>      default 480\n"
        "  --viewer-follow-body <b>  camera follow body (default pelvis)\n"
        "  --help\n",
        program);
}

Config parseArgs(int argc, char** argv) {
    Config cfg;
    namespace fs = std::filesystem;
    const fs::path executable_dir = fs::canonical(argv[0]).parent_path();
    const fs::path repo_root = executable_dir.parent_path();
    cfg.xml_file = (repo_root / "assets/e1/mjcf/e1_24dof.xml").string();
    cfg.ik_config =
        (repo_root / "config/ik_configs/gem_to_e1_position.json").string();

    bool explicit_key = false;
    std::string requested_key;
    double pelvis_z_offset = 0.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (++i >= argc)
                throw std::runtime_error("missing value for " + arg);
            return argv[i];
        };

        if      (arg == "--bind")              cfg.bind_ip = next();
        else if (arg == "--port")              cfg.port = std::stoi(next());
        else if (arg == "--gem-protocol") {
            const std::string value = next();
            if (value == "auto") cfg.gem_protocol = gmr::GemProtocol::Any;
            else if (value == "gem1") cfg.gem_protocol = gmr::GemProtocol::Gem1;
            else if (value == "gem2") cfg.gem_protocol = gmr::GemProtocol::Gem2;
            else throw std::runtime_error(
                "--gem-protocol must be auto, gem1, or gem2");
        }
        else if (arg == "--xml")               cfg.xml_file = next();
        else if (arg == "--ik-config")         cfg.ik_config = next();
        else if (arg == "--preset")            cfg.preset = next();
        else if (arg == "--human-height")      cfg.human_height = std::stod(next());
        else if (arg == "--damping")           cfg.damping = std::stod(next());
        else if (arg == "--redis-host")        cfg.redis.host = next();
        else if (arg == "--redis-port")        cfg.redis.port = std::stoi(next());
        else if (arg == "--redis-db")          cfg.redis.db = std::stoi(next());
        else if (arg == "--redis-key")         { requested_key = next(); explicit_key = true; }
        else if (arg == "--hz")                cfg.publish_hz = std::stod(next());
        else if (arg == "--ttl-ms")            cfg.redis.ttl_ms = std::stoi(next());
        else if (arg == "--stale-ms")          cfg.stale_ms = std::stod(next());
        else if (arg == "--lin-vel-alpha")     cfg.redis.lin_vel_alpha = std::stod(next());
        else if (arg == "--ang-vel-alpha")     cfg.redis.ang_vel_alpha = std::stod(next());
        else if (arg == "--lin-vel-max")       cfg.redis.lin_vel_max = std::stod(next());
        else if (arg == "--ang-vel-max")       cfg.redis.ang_vel_max = std::stod(next());
        else if (arg == "--pelvis-z-offset")   pelvis_z_offset = std::stod(next());
        else if (arg == "--offset-to-ground")  cfg.offset_to_ground = true;
        else if (arg == "--no-offset-to-ground") cfg.offset_to_ground = false;
        else if (arg == "--always")            cfg.require_buttons = false;
        else if (arg == "--vis")               cfg.vis = true;
        else if (arg == "--vis-smpl-targets")  {
            cfg.vis_smpl_targets = true;
            cfg.vis = true;
        }
        else if (arg == "--viewer-width")      cfg.viewer_width = std::stoi(next());
        else if (arg == "--viewer-height")     cfg.viewer_height = std::stoi(next());
        else if (arg == "--viewer-follow-body") cfg.viewer_follow_body = next();
        else if (arg == "--help" || arg == "-h") {
            usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (cfg.preset != "e1")
        throw std::runtime_error("GEM server only supports --preset e1");
    cfg.redis.applyPreset(gmr::presetE1());

    if (explicit_key) cfg.redis.key = requested_key;
    cfg.redis.pelvis_z_offset = pelvis_z_offset;

    if (cfg.publish_hz <= 0.0) throw std::runtime_error("--hz must be > 0");
    if (cfg.stale_ms <= 0.0)   throw std::runtime_error("--stale-ms must be > 0");
    if (!fs::is_regular_file(cfg.xml_file))
        throw std::runtime_error("E1 XML not found: " + cfg.xml_file);
    if (!fs::is_regular_file(cfg.ik_config))
        throw std::runtime_error("E1 IK config not found: " + cfg.ik_config);
    if (cfg.redis.ttl_ms <= 0)
        std::cerr << "[warn] --ttl-ms <= 0: stale input will stop updates, "
                     "but the last Redis SET value will not expire\n";
    return cfg;
}

double steadyNowSec() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    Config cfg;
    try {
        cfg = parseArgs(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "[fatal] " << e.what() << "\n";
        usage(argv[0]);
        return 2;
    }

    try {
        std::printf(
            "[Config] preset=e1\n"
            "[Config] xml=%s\n"
            "[Config] ik=%s\n"
            "[Config] redis=%s:%d/%d key=%s\n"
            "[Config] joints=24 frame=38 floats / 152 bytes\n"
            "[Config] bind=%s:%d protocol=%s stale_ms=%.0f "
            "offset_to_ground=%s vis_smpl_targets=%s\n",
            cfg.xml_file.c_str(), cfg.ik_config.c_str(),
            cfg.redis.host.c_str(), cfg.redis.port, cfg.redis.db,
            cfg.redis.key.c_str(), cfg.bind_ip.c_str(), cfg.port,
            gmr::GemReader::protocolName(cfg.gem_protocol), cfg.stale_ms,
            cfg.offset_to_ground ? "on" : "off",
            cfg.vis_smpl_targets ? "on" : "off");

        gmr::FrameQueue queue(300);

        gmr::GemReader::Config reader_cfg;
        reader_cfg.bind_ip = cfg.bind_ip;
        reader_cfg.port = cfg.port;
        reader_cfg.verbose = true;
        reader_cfg.expected_protocol = cfg.gem_protocol;
        gmr::GemReader reader(queue, reader_cfg);
        reader.connect();

        gmr_mink::GMR gmr(
            cfg.xml_file, cfg.ik_config, cfg.human_height, cfg.damping, false);
        std::cout << "[GMR] ready; waiting for GEM frames...\n";

        constexpr int    kSeedFrames = 10;
        constexpr double kFrameTimeoutSec = 0.02;
        constexpr double kMaxBufferSec = 12.0;
        const size_t max_frames =
            std::max<size_t>(static_cast<size_t>(
                kMaxBufferSec / kFrameTimeoutSec), 32);

        gmr::MotionBuffer buffer(max_frames, kFrameTimeoutSec);
        buffer.setOffsetToGround(cfg.offset_to_ground);
        buffer.setCaptureTargetData(cfg.vis_smpl_targets);

        std::cout << "[Init] collecting " << kSeedFrames << " seed frames...\n";
        buffer.seedSync(kSeedFrames, queue, &gmr);

        // Re-warm with a real GEM pose while preserving the solver's warm state.
        const gmr::BodyMap warm_body = buffer.latestBodyData();
        if (!warm_body.empty()) {
            for (int i = 0; i < 1000; ++i)
                gmr.retarget(warm_body, cfg.offset_to_ground);
        }

        // Remove datagrams accumulated while re-warming; start from the newest stream.
        {
            gmr::RawFrame stale;
            int flushed = 0;
            while (queue.pop(stale, 0.0)) ++flushed;
            if (flushed)
                std::printf("[Init] flushed %d queued warm-up frames\n", flushed);
        }

        buffer.clear();
        buffer.startAsync(queue, &gmr);

        std::unique_ptr<gmr::MujocoViewer> viewer;
        if (cfg.vis)
            viewer = std::make_unique<gmr::MujocoViewer>(
                cfg.xml_file, cfg.viewer_width, cfg.viewer_height,
                cfg.viewer_follow_body);

        const auto period =
            std::chrono::duration<double>(1.0 / cfg.publish_hz);

        std::thread publisher_thread([&] {
            gmr::RedisPublisher publisher(cfg.redis);
            publisher.clearKey();
            Joystick joystick;

            auto next_tick = std::chrono::steady_clock::now() + period;
            auto last_stale_warning =
                std::chrono::steady_clock::now() - std::chrono::seconds(1);
            const bool diag = std::getenv("GMR_PUB_DIAG") != nullptr;
            auto diag_t0 = std::chrono::steady_clock::now();
            uint64_t ticks = 0, sent = 0, stale_ticks = 0, empty_ticks = 0;
            uint64_t gate_ticks = 0;

            while (!g_stop) {
                ++ticks;
                joystick.update();
                if (cfg.require_buttons &&
                    !(joystick.pressed(Joystick::kButtonA) &&
                      joystick.pressed(Joystick::kButtonR1))) {
                    ++gate_ticks;
                    std::this_thread::sleep_until(next_tick);
                    next_tick += period;
                    continue;
                }

                const bool received = reader.hasReceivedFrame();
                const double input_age_ms = reader.lastReceiveAgeMs();

                if (!received || input_age_ms > cfg.stale_ms) {
                    ++stale_ticks;
                    const auto warning_now = std::chrono::steady_clock::now();
                    if (warning_now - last_stale_warning >=
                        std::chrono::seconds(1)) {
                        std::fprintf(stderr,
                            "[Safety] GEM input stale (%.1f ms); Redis refresh paused. "
                            "The current key will expire via TTL.\n",
                            input_age_ms);
                        last_stale_warning = warning_now;
                    }
                } else {
                    auto frame = buffer.latestProcessedFrame();
                    if (frame && frame->qpos.size() > 0) {
                        // frame_time is GemReader's local steady receive time.
                        const double frame_age_ms =
                            (steadyNowSec() - frame->frame_time) * 1000.0;
                        if (frame_age_ms <= cfg.stale_ms) {
                            if (publisher.publish(
                                    frame->qpos, frame->frame_time,
                                    frame->body_data)) {
                                ++sent;
                            }
                        } else {
                            ++stale_ticks;
                        }
                    } else {
                        ++empty_ticks;
                    }
                }

                std::this_thread::sleep_until(next_tick);
                next_tick += period;
                const auto now = std::chrono::steady_clock::now();
                if (now > next_tick + period)
                    next_tick = now + period;

                const double sec =
                    std::chrono::duration<double>(now - diag_t0).count();
                if (diag && sec >= 5.0) {
                    std::printf(
                        "[GemPubDiag] tick=%.1fHz sent=%.1fHz input=%.1fms "
                        "stale=%llu gate=%llu empty=%llu buffer=%zu recv=%llu reject=%llu drop=%llu\n",
                        ticks / sec, sent / sec, reader.lastReceiveAgeMs(),
                        static_cast<unsigned long long>(stale_ticks),
                        static_cast<unsigned long long>(gate_ticks),
                        static_cast<unsigned long long>(empty_ticks),
                        buffer.length(),
                        static_cast<unsigned long long>(reader.packetsReceived()),
                        static_cast<unsigned long long>(reader.packetsInvalid()),
                        static_cast<unsigned long long>(reader.packetsDropped()));
                    diag_t0 = now;
                    ticks = sent = stale_ticks = empty_ticks = gate_ticks = 0;
                }
            }
        });

        std::printf(
            "[Run] GEM UDP=%s:%d preset=%s Redis=%s:%d/%d key=%s "
            "publish=%.1fHz stale=%.0fms ttl=%dms offset_to_ground=%s "
            "gate=%s vis=%s targets=%s\n",
            cfg.bind_ip.c_str(), cfg.port, cfg.preset.c_str(),
            cfg.redis.host.c_str(), cfg.redis.port, cfg.redis.db,
            cfg.redis.key.c_str(), cfg.publish_hz, cfg.stale_ms,
            cfg.redis.ttl_ms,
            cfg.offset_to_ground ? "on" : "off",
            cfg.require_buttons ? "A+R1" : "off",
            cfg.vis ? "on" : "off",
            cfg.vis_smpl_targets ? "on" : "off");

        while (!g_stop) {
            const auto t0 = std::chrono::steady_clock::now();
            if (viewer) {
                auto frame = buffer.latestProcessedFrame();
                if (frame && frame->qpos.size() > 0) {
                    if (cfg.vis_smpl_targets) {
                        viewer->render(
                            frame->qpos, &frame->body_data, &frame->target_data);
                    } else {
                        viewer->render(frame->qpos);
                    }
                }
                if (viewer->shouldClose()) break;
                std::this_thread::sleep_until(
                    t0 + std::chrono::milliseconds(5));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

        g_stop = true;
        if (publisher_thread.joinable()) publisher_thread.join();
        buffer.stopAsync();
        reader.disconnect();
        std::cout << "[Run] stopped.\n";
    } catch (const std::exception& e) {
        g_stop = true;
        std::cerr << "[fatal] " << e.what() << "\n";
        return 1;
    }

    return 0;
}
