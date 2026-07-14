/**
 * pico_mocap_server.cpp - Pico -> GMR -> Redis server
 *
 * Pico uses the same retarget/publish pipeline as OptiTrack, Xsens, and
 * FZMotion. Only the reader is device-specific.
 */

#include "gmr/frame_queue.hpp"
#include "gmr/motion_buffer.hpp"
#include "gmr/mujoco_viewer.hpp"
#include "gmr/redis_publisher.hpp"
#include "gmr/gmr_mink.hpp"

#ifndef USE_DUMMY_READER
#define USE_DUMMY_READER 0
#endif
#if USE_DUMMY_READER
#include "readers/dummy_reader.hpp"
using ActiveReader = gmr::DummyReader;
#else
#include "readers/pico_reader.hpp"
using ActiveReader = gmr::PicoReader;
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>

std::atomic<bool> g_stop{false};
static void sigHandler(int) { g_stop = true; }

struct Config {
    std::string xml_file;
    std::string ik_config_path;
    std::string preset = "g1";
    gmr::RedisPublisher::Config redis;
    double publish_hz = 50.0;
    double actual_human_height = 1.6;
    bool   vis = false;
    int    viewer_width = 640;
    int    viewer_height = 480;
    bool   offset_to_ground = true;

    // Accepted for run_pico.sh compatibility; the unified pipeline does not
    // use a separate GMR clock or interpolation buffer.
    double pico_hz = 80.0;
    double gmr_hz = 30.0;
    double buffer_ms = 200.0;
};

static void usage(const char* p) {
    std::printf(
        "Usage: %s --xml <xml> --ik-config <json> [opts]\n"
        "  --xml             MuJoCo XML path\n"
        "  --ik-config       IK config JSON path\n"
        "  --preset          Robot preset: g1 (default), e1\n"
        "  --height          actual human height in meters (default 1.6)\n"
        "  --redis-host      default 127.0.0.1\n"
        "  --redis-port      default 6379\n"
        "  --redis-db        default 0\n"
        "  --redis-key       override Redis key\n"
        "  --hz              publish rate Hz (default 50)\n"
        "  --ttl-ms          Redis TTL ms (default 200)\n"
        "  --lin-vel-alpha   linear velocity EMA alpha (default 1=no filter)\n"
        "  --ang-vel-alpha   angular velocity EMA alpha (default 1=no filter)\n"
        "  --lin-vel-max     reject linear velocity spikes above this m/s (default 0=disabled)\n"
        "  --ang-vel-max     reject angular velocity spikes above this rad/s (default 0=disabled)\n"
        "  --vis             open MuJoCo viewer\n"
        "  --viewer-width    viewer render width (default 640)\n"
        "  --viewer-height   viewer render height (default 480)\n"
        "  --no-offset-to-ground  disable offset to ground (default: enabled)\n"
        "  --no-raw-bones    disable per-frame raw bones JSON publishing\n"
        "\n"
        "Compatibility options accepted but not used by this unified path:\n"
        "  --pico-hz --gmr-hz --buffer-ms --min-buffer-ms --vel-lpf-alpha --lin-vel-scale\n",
        p);
}

static Config parseArgs(int argc, char** argv) {
    Config cfg;
    bool explicit_key = false;
    std::string explicit_redis_key;

    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        auto nxt = [&]() -> std::string {
            if (++i >= argc) throw std::runtime_error("missing value for " + s);
            return argv[i];
        };

        if      (s == "--xml")             cfg.xml_file              = nxt();
        else if (s == "--ik-config")       cfg.ik_config_path        = nxt();
        else if (s == "--preset")          cfg.preset                = nxt();
        else if (s == "--height")          cfg.actual_human_height   = std::stod(nxt());
        else if (s == "--redis-host")      cfg.redis.host            = nxt();
        else if (s == "--redis-port")      cfg.redis.port            = std::stoi(nxt());
        else if (s == "--redis-db")        cfg.redis.db              = std::stoi(nxt());
        else if (s == "--redis-key")       {
            cfg.redis.key = nxt();
            explicit_redis_key = cfg.redis.key;
            explicit_key = true;
        }
        else if (s == "--hz")              cfg.publish_hz            = std::stod(nxt());
        else if (s == "--ttl-ms")          cfg.redis.ttl_ms          = std::stoi(nxt());
        else if (s == "--lin-vel-alpha")   cfg.redis.lin_vel_alpha   = std::stod(nxt());
        else if (s == "--ang-vel-alpha")   cfg.redis.ang_vel_alpha   = std::stod(nxt());
        else if (s == "--lin-vel-max")     cfg.redis.lin_vel_max     = std::stod(nxt());
        else if (s == "--ang-vel-max")     cfg.redis.ang_vel_max     = std::stod(nxt());
        else if (s == "--vis")             cfg.vis                   = true;
        else if (s == "--viewer-width")    cfg.viewer_width          = std::stoi(nxt());
        else if (s == "--viewer-height")   cfg.viewer_height         = std::stoi(nxt());
        else if (s == "--offset-to-ground") cfg.offset_to_ground     = true;
        else if (s == "--no-offset-to-ground") cfg.offset_to_ground  = false;
        else if (s == "--raw-bones")       cfg.redis.publish_raw_bones = true;
        else if (s == "--no-raw-bones")    cfg.redis.publish_raw_bones = false;
        else if (s == "--always")          (void)0;
        else if (s == "--pico-hz")         cfg.pico_hz               = std::stod(nxt());
        else if (s == "--gmr-hz")          cfg.gmr_hz                = std::stod(nxt());
        else if (s == "--buffer-ms")       cfg.buffer_ms             = std::stod(nxt());
        else if (s == "--min-buffer-ms")   (void)nxt();
        else if (s == "--vel-lpf-alpha")   (void)nxt();
        else if (s == "--lin-vel-scale")   (void)nxt();
        else if (s == "--help")            { usage(argv[0]); exit(0); }
        else std::cerr << "[warn] unknown: " << s << "\n";
    }

    if (cfg.preset == "e1") {
        cfg.redis.applyPreset(gmr::presetE1());
    } else {
        cfg.preset = "g1";
        cfg.redis.applyPreset(gmr::presetG1());
    }
    if (explicit_key) cfg.redis.key = explicit_redis_key;

    if (cfg.xml_file.empty() || cfg.ik_config_path.empty()) {
        usage(argv[0]);
        exit(1);
    }
    return cfg;
}

int main(int argc, char** argv) {
    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);

    Config cfg;
    try {
        cfg = parseArgs(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "[fatal] " << e.what() << "\n";
        return 1;
    }

    try {
        gmr::FrameQueue queue;

        ActiveReader reader(queue);
        reader.connect();

        gmr_mink::GMR gmr(cfg.xml_file, cfg.ik_config_path,
                          cfg.actual_human_height, 0.5, false);
        std::cout << "[GMR] ready.\n";

        static constexpr int    LOOKAHEAD   = 2;
        static constexpr int    SEED_FRAMES = 10;
        static constexpr double MAX_BUF_SEC = 12.0;
        static constexpr double FRAME_TO    = 0.02;

        size_t max_frames = std::max<size_t>(
            size_t(MAX_BUF_SEC / FRAME_TO), 2 * LOOKAHEAD + 10);

        gmr::MotionBuffer buf(max_frames, FRAME_TO);
        buf.setOffsetToGround(cfg.offset_to_ground);

        std::cout << "[Init] seeding buffer...\n";
        buf.seedSync(SEED_FRAMES, queue, &gmr);
        std::cout << "[Init] seed done.\n";

        {
            gmr::RawFrame first;
            bool got = false;
            for (int i = 0; i < 50 && !g_stop; ++i)
                if (queue.pop(first, 0.1)) { got = true; break; }
            if (got) {
                for (int i = 0; i < 1000; ++i)
                    gmr.retarget(first.body_data, cfg.offset_to_ground);
                std::cout << "[GMR] re-warm done.\n";
            }
        }

        {
            gmr::RawFrame dummy;
            int flushed = 0;
            while (queue.pop(dummy, 0.0)) flushed++;
            if (flushed > 0)
                std::printf("[Init] flushed %d stale frames after warmup\n", flushed);
        }

        buf.clear();
        buf.startAsync(queue, &gmr);

        std::unique_ptr<gmr::MujocoViewer> viewer;
        if (cfg.vis)
            viewer = std::make_unique<gmr::MujocoViewer>(
                cfg.xml_file, cfg.viewer_width, cfg.viewer_height);

        const auto period = std::chrono::duration<double>(1.0 / cfg.publish_hz);
        std::thread pub_thread([&] {
            gmr::RedisPublisher pub(cfg.redis);
            pub.clearKey();
            auto next_tick = std::chrono::steady_clock::now() + period;

            const bool diag = std::getenv("GMR_PUB_DIAG") != nullptr;
            auto diag_t0 = std::chrono::steady_clock::now();
            long long ticks = 0, sent = 0, pub_skip = 0;
            long long buf_skip = 0, empty_skip = 0;
            double publish_ms_sum = 0.0;

            while (!g_stop) {
                ticks++;

                if (buf.length() < LOOKAHEAD + 1) {
                    buf_skip++;
                    std::this_thread::sleep_until(next_tick);
                    next_tick += period;
                    continue;
                }

                auto frame = buf.latestProcessedFrame();
                if (frame && frame->qpos.size() > 0) {
                    auto p0 = std::chrono::steady_clock::now();
                    bool ok = pub.publish(frame->qpos, frame->frame_time, frame->body_data);
                    auto p1 = std::chrono::steady_clock::now();
                    publish_ms_sum += std::chrono::duration<double, std::milli>(p1 - p0).count();
                    if (ok) sent++;
                    else pub_skip++;
                } else {
                    empty_skip++;
                }

                std::this_thread::sleep_until(next_tick);
                next_tick += period;
                auto now = std::chrono::steady_clock::now();
                if (now > next_tick + period)
                    next_tick = now + period;

                if (diag && std::chrono::duration<double>(now - diag_t0).count() >= 5.0) {
                    double sec = std::chrono::duration<double>(now - diag_t0).count();
                    double avg_ms = (sent + pub_skip) > 0
                        ? publish_ms_sum / double(sent + pub_skip) : 0.0;
                    std::printf("[PubLoopDiag] tick=%.1fHz sent=%.1fHz "
                                "buf_skip=%lld empty=%lld pub_skip=%lld "
                                "avg_publish=%.3fms buf_len=%zu\n",
                                ticks / sec, sent / sec,
                                buf_skip, empty_skip, pub_skip,
                                avg_ms, buf.length());
                    diag_t0 = now;
                    ticks = sent = pub_skip = buf_skip = empty_skip = 0;
                    publish_ms_sum = 0.0;
                }
            }
        });

        std::printf("[Run] pico unified pipeline preset=%s hz=%.1f vis=%s viewer=%dx%d height=%.2f "
                    "offset_to_ground=%s "
                    "compat(pico_hz=%.0f gmr_hz=%.0f buffer_ms=%.0f)\n",
                    cfg.preset.c_str(),
                    cfg.publish_hz,
                    cfg.vis ? "on" : "off",
                    cfg.viewer_width,
                    cfg.viewer_height,
                    cfg.actual_human_height,
                    cfg.offset_to_ground ? "on" : "off",
                    cfg.pico_hz,
                    cfg.gmr_hz,
                    cfg.buffer_ms);

        while (!g_stop) {
            auto t0 = std::chrono::steady_clock::now();
            if (viewer) {
                auto frame = buf.latestProcessedFrame();
                if (frame && frame->qpos.size() > 0)
                    viewer->render(frame->qpos);
                if (viewer->shouldClose()) break;
                std::this_thread::sleep_until(t0 + std::chrono::milliseconds(5));
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

        g_stop = true;
        if (pub_thread.joinable()) pub_thread.join();
        buf.stopAsync();
        reader.disconnect();
        std::cout << "[Run] stopped.\n";

    } catch (const std::exception& e) {
        std::cerr << "[fatal] " << e.what() << "\n";
        return 1;
    }
    return 0;
}
