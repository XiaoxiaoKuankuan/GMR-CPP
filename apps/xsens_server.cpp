/**
 * xsens_server.cpp — Xsens MVN → GMR → Redis server
 *
 * Identical to mocap_server.cpp except:
 *   - Uses XsensReader instead of OptiTrackReader
 *   - --server/--client replaced with --port
 *   - --offset-to-ground / --no-offset-to-ground added
 *
 * mocap_server.cpp is untouched.
 */

#include "gmr/frame_queue.hpp"
#include "gmr/motion_buffer.hpp"
#include "gmr/mujoco_viewer.hpp"
#include "gmr/redis_publisher.hpp"
#include "gmr/gmr_mink.hpp"
#include "readers/xsens_reader.hpp"

#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#include <filesystem>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>

std::atomic<bool> g_stop{false};
static void sigHandler(int) { g_stop = true; }

// ── Joystick ──────────────────────────────────────────────────────────────────
struct Joystick {
    static constexpr int JOY_BTN_A  = 0;
    static constexpr int JOY_BTN_R1 = 5;
    static constexpr int MAX_BTN    = 32;

    Joystick() {
        fd_ = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
        if (fd_ < 0)
            std::cerr << "[Joystick] not found, running without gate\n";
        std::memset(btns_, 0, sizeof(btns_));
    }
    ~Joystick() { if (fd_ >= 0) close(fd_); }

    void update() {
        if (fd_ < 0) return;
        js_event ev;
        while (read(fd_, &ev, sizeof(ev)) == sizeof(ev))
            if ((ev.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON && ev.number < MAX_BTN)
                btns_[ev.number] = ev.value;
    }
    bool pressed(int b) const { return fd_ >= 0 && b < MAX_BTN && btns_[b]; }

private:
    int fd_ = -1;
    int btns_[MAX_BTN];
};

// ── Spine1 pitch offset ───────────────────────────────────────────────────────
static void applySpine1PitchOffset(gmr::BodyMap& data,
                                   const std::string& joint,
                                   double degrees)
{
    if (joint.empty() || degrees == 0.0) return;
    auto it = data.find(joint);
    if (it == data.end()) return;
    Eigen::Vector4d& q = it->second.rot_wxyz;
    Eigen::Quaterniond eq(q[0], q[1], q[2], q[3]);
    Eigen::Quaterniond off(Eigen::AngleAxisd(degrees * M_PI / 180.0,
                                              Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond res = eq * off;
    q = Eigen::Vector4d(res.w(), res.x(), res.y(), res.z());
}

// ── Config ────────────────────────────────────────────────────────────────────
struct Config {
    int         xsens_port      = 9763;
    std::string xml_file;
    std::string ik_config_path;
    std::string preset          = "g1";
    gmr::RedisPublisher::Config redis;
    double      publish_hz      = 50.0;
    bool        require_buttons = true;
    bool        vis             = false;
    int         viewer_width     = 640;
    int         viewer_height    = 480;
    std::string spine_joint     = "";
    double      spine_deg       = 0.0;
    bool        offset_to_ground = true;  // default true，和 Python GMR 一致
};

static void usage(const char* p) {
    std::printf(
        "Usage: %s [opts]\n"
        "  --port              Xsens MVN UDP port (default 9763)\n"
        "  --xml               MuJoCo XML path\n"
        "  --ik-config         IK config JSON path\n"
        "  --preset            Robot preset: g1 (default), e1\n"
        "  --redis-host        default 127.0.0.1\n"
        "  --redis-port        default 6379\n"
        "  --redis-db          default 0\n"
        "  --redis-key         override Redis key\n"
        "  --hz                publish rate Hz (default 50)\n"
        "  --ttl-ms            Redis TTL ms (default 200)\n"
        "  --lin-vel-alpha     linear velocity EMA alpha (default 1=no filter)\n"
        "  --ang-vel-alpha     angular velocity EMA alpha (default 1=no filter)\n"
        "  --lin-vel-max       reject linear velocity spikes above this m/s (default 0=disabled)\n"
        "  --ang-vel-max       reject angular velocity spikes above this rad/s (default 0=disabled)\n"
        "  --always            publish without joystick gate\n"
        "  --vis               open MuJoCo viewer\n"
        "  --viewer-width      viewer render width (default 640)\n"
        "  --viewer-height     viewer render height (default 480)\n"
        "  --no-spine-offset   disable spine pitch offset\n"
        "  --no-offset-to-ground  disable offset to ground (default: enabled)\n", p);
}

static Config parseArgs(int argc, char** argv) {
    Config cfg;
    namespace fs = std::filesystem;
    fs::path here = fs::canonical(argv[0]).parent_path();

    bool explicit_key   = false;
    bool no_spine       = false;
    bool explicit_spine = false;

    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        auto nxt = [&]() -> std::string {
            if (++i >= argc) throw std::runtime_error("missing value for " + s);
            return argv[i];
        };
        if      (s=="--port")            cfg.xsens_port         = std::stoi(nxt());
        else if (s=="--xml")             cfg.xml_file           = nxt();
        else if (s=="--ik-config")       cfg.ik_config_path     = nxt();
        else if (s=="--preset")          cfg.preset             = nxt();
        else if (s=="--redis-host")      cfg.redis.host         = nxt();
        else if (s=="--redis-port")      cfg.redis.port         = std::stoi(nxt());
        else if (s=="--redis-db")        cfg.redis.db           = std::stoi(nxt());
        else if (s=="--redis-key")       { cfg.redis.key        = nxt(); explicit_key = true; }
        else if (s=="--hz")              cfg.publish_hz         = std::stod(nxt());
        else if (s=="--ttl-ms")          cfg.redis.ttl_ms       = std::stoi(nxt());
        else if (s=="--lin-vel-alpha")   cfg.redis.lin_vel_alpha = std::stod(nxt());
        else if (s=="--ang-vel-alpha")   cfg.redis.ang_vel_alpha = std::stod(nxt());
        else if (s=="--lin-vel-max")     cfg.redis.lin_vel_max   = std::stod(nxt());
        else if (s=="--ang-vel-max")     cfg.redis.ang_vel_max   = std::stod(nxt());
        else if (s=="--always")          cfg.require_buttons    = false;
        else if (s=="--vis")             cfg.vis                = true;
        else if (s=="--viewer-width")    cfg.viewer_width       = std::stoi(nxt());
        else if (s=="--viewer-height")   cfg.viewer_height      = std::stoi(nxt());
        else if (s=="--spine-joint")     { cfg.spine_joint      = nxt(); explicit_spine = true; }
        else if (s=="--spine-deg")       { cfg.spine_deg        = std::stod(nxt()); explicit_spine = true; }
        else if (s=="--no-spine-offset") no_spine               = true;
        else if (s=="--no-offset-to-ground") cfg.offset_to_ground = false;
        else if (s=="--help")            { usage(argv[0]); exit(0); }
        else std::cerr << "[warn] unknown arg: " << s << "\n";
    }

    if (cfg.preset == "e1") {
        cfg.redis.applyPreset(gmr::presetE1());
        if (cfg.xml_file.empty())
            cfg.xml_file = (here / "../../assets/unitree_e1/e1_mocap.xml").string();
        if (cfg.ik_config_path.empty())
            cfg.ik_config_path = (here / "../../config/ik_configs/fbx_to_e1.json").string();
        if (!explicit_spine && !no_spine) { cfg.spine_joint = ""; cfg.spine_deg = 0.0; }
    } else {
        cfg.preset = "g1";
        cfg.redis.applyPreset(gmr::presetG1());
        if (cfg.xml_file.empty())
            cfg.xml_file = (here / "../../assets/unitree_g1/g1_mocap_29dof.xml").string();
        if (cfg.ik_config_path.empty())
            cfg.ik_config_path = (here / "../../config/ik_configs/fbx_to_g1.json").string();
        if (!explicit_spine && !no_spine) { cfg.spine_joint = "Spine1"; cfg.spine_deg = 6.0; }
    }

    if (no_spine) { cfg.spine_joint = ""; cfg.spine_deg = 0.0; }
    (void)explicit_key;
    cfg.redis.pelvis_z_offset = 0.0;  // ← 加这一行
    return cfg;
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);

    Config cfg;
    try { cfg = parseArgs(argc, argv); }
    catch (const std::exception& e) {
        std::cerr << "[fatal] " << e.what() << "\n"; return 1;
    }

    const std::string spine_joint = cfg.spine_joint;
    const double      spine_deg   = cfg.spine_deg;
    const bool        spine_on    = !spine_joint.empty() && spine_deg != 0.0;

    try {
        gmr::FrameQueue queue;

        // ── XsensReader ───────────────────────────────────────────────────
        gmr::XsensReader::Config rc;
        rc.port    = cfg.xsens_port;
        rc.verbose = true;
        gmr::XsensReader reader(queue, rc);
        reader.connect();

        // ── GMR ───────────────────────────────────────────────────────────
        gmr_mink::GMR gmr(cfg.xml_file, cfg.ik_config_path, 1.8, 1.0, false);
        std::cout << "[GMR] ready.\n";

        // ── MotionBuffer ──────────────────────────────────────────────────
        static constexpr int    LOOKAHEAD   = 2;
        static constexpr int    SEED_FRAMES = 10;
        static constexpr double MAX_BUF_SEC = 12.0;
        static constexpr double FRAME_TO    = 0.02;

        size_t max_frames = std::max<size_t>(
            size_t(MAX_BUF_SEC / FRAME_TO), 2*LOOKAHEAD+10);

        gmr::MotionBuffer buf(max_frames, FRAME_TO);

        if (spine_on) {
            buf.setPreprocessFn([spine_joint, spine_deg](gmr::BodyMap& m) {
                applySpine1PitchOffset(m, spine_joint, spine_deg);
            });
        }
        buf.setOffsetToGround(cfg.offset_to_ground);

        std::cout << "[Init] seeding buffer...\n";
        buf.seedSync(SEED_FRAMES, queue, &gmr);
        std::cout << "[Init] seed done.\n";

        // Re-warm IK
        {
            gmr::RawFrame first;
            bool got = false;
            for (int i = 0; i < 50 && !g_stop; ++i)
                if (queue.pop(first, 0.1)) { got = true; break; }
            if (got) {
                if (spine_on)
                    applySpine1PitchOffset(first.body_data, spine_joint, spine_deg);
                for (int i = 0; i < 1000; ++i)
                    gmr.retarget(first.body_data, cfg.offset_to_ground);
                std::cout << "[GMR] re-warm done.\n";
            }
        }

        // 清空 warmup 期间积压的旧帧
        {
            gmr::RawFrame dummy;
            int flushed = 0;
            while (queue.pop(dummy, 0.0)) flushed++;
            if (flushed > 0)
                std::printf("[Init] flushed %d stale frames after warmup\n", flushed);
        }

        buf.clear();
        buf.startAsync(queue, &gmr);

        // ── MuJoCo viewer ─────────────────────────────────────────────────
        std::unique_ptr<gmr::MujocoViewer> viewer;
        if (cfg.vis)
            viewer = std::make_unique<gmr::MujocoViewer>(
                cfg.xml_file, cfg.viewer_width, cfg.viewer_height);

        // ── Redis publisher thread ────────────────────────────────────────
        const auto period = std::chrono::duration<double>(1.0 / cfg.publish_hz);
        std::thread pub_thread([&] {
            gmr::RedisPublisher pub(cfg.redis);
            pub.clearKey();
            Joystick joy;
            auto next_tick = std::chrono::steady_clock::now() + period;
            const bool diag = std::getenv("GMR_PUB_DIAG") != nullptr;
            auto diag_t0 = std::chrono::steady_clock::now();
            long long ticks = 0, sent = 0, pub_skip = 0;
            long long gate_skip = 0, buf_skip = 0, empty_skip = 0;
            double publish_ms_sum = 0.0;
            bool have_last_diag_frame = false;
            double last_diag_frame_time = 0.0;
            Eigen::VectorXd last_diag_qpos;
            long long new_frames = 0;
            long long hold_ticks = 0;
            long long hold_streak = 0;
            long long max_hold_streak = 0;
            long long out_of_order = 0;
            double max_new_dt_ms = 0.0;
            double max_root_jump = 0.0;
            double max_joint_jump = 0.0;

            while (!g_stop) {
                ticks++;
                joy.update();

                if (cfg.require_buttons) {
                    bool on = joy.pressed(Joystick::JOY_BTN_A) &&
                              joy.pressed(Joystick::JOY_BTN_R1);
                    if (!on) {
                        gate_skip++;
                        std::this_thread::sleep_until(next_tick);
                        next_tick += period;
                        continue;
                    }
                }

                if (buf.length() < LOOKAHEAD + 1) {
                    buf_skip++;
                    std::this_thread::sleep_until(next_tick);
                    next_tick += period;
                    continue;
                }

                auto frame = buf.latestProcessedFrame();
                if (frame && frame->qpos.size() > 0) {
                    const double ft = frame->frame_time;
                    const bool duplicate_frame =
                        have_last_diag_frame && std::abs(ft - last_diag_frame_time) <= 1e-6;
                    const bool time_reversed =
                        have_last_diag_frame && ft < last_diag_frame_time - 1e-6;

                    if (duplicate_frame) {
                        hold_ticks++;
                        hold_streak++;
                        max_hold_streak = std::max(max_hold_streak, hold_streak);
                    } else if (time_reversed) {
                        out_of_order++;
                    } else {
                        new_frames++;
                        if (have_last_diag_frame) {
                            max_new_dt_ms = std::max(
                                max_new_dt_ms, (ft - last_diag_frame_time) * 1000.0);

                            const int n = std::min<int>(last_diag_qpos.size(), frame->qpos.size());
                            if (n >= 3) {
                                max_root_jump = std::max(
                                    max_root_jump,
                                    (frame->qpos.head<3>() - last_diag_qpos.head<3>()).norm());
                            }
                            if (n > 7) {
                                double joint_jump = 0.0;
                                for (int qi = 7; qi < n; ++qi) {
                                    joint_jump = std::max(
                                        joint_jump,
                                        std::abs(frame->qpos[qi] - last_diag_qpos[qi]));
                                }
                                max_joint_jump = std::max(max_joint_jump, joint_jump);
                            }
                        }
                        last_diag_frame_time = ft;
                        last_diag_qpos = frame->qpos;
                        have_last_diag_frame = true;
                        hold_streak = 0;
                    }

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
                    double avg_ms = (sent + pub_skip) > 0 ? publish_ms_sum / double(sent + pub_skip) : 0.0;
                    std::printf("[PubLoopDiag] tick=%.1fHz sent=%.1fHz gate_skip=%lld "
                                "buf_skip=%lld empty=%lld pub_skip=%lld avg_publish=%.3fms "
                                "buf_len=%zu new=%.1fHz holds=%lld max_hold=%.1fms "
                                "max_new_dt=%.1fms max_root_jump=%.4fm max_joint_jump=%.4frad "
                                "ooo=%lld\n",
                                ticks / sec, sent / sec, gate_skip,
                                buf_skip, empty_skip, pub_skip, avg_ms, buf.length(),
                                new_frames / sec, hold_ticks,
                                max_hold_streak * 1000.0 / cfg.publish_hz,
                                max_new_dt_ms, max_root_jump, max_joint_jump,
                                out_of_order);
                    diag_t0 = now;
                    ticks = sent = pub_skip = gate_skip = buf_skip = empty_skip = 0;
                    new_frames = hold_ticks = out_of_order = 0;
                    max_hold_streak = hold_streak;
                    max_new_dt_ms = max_root_jump = max_joint_jump = 0.0;
                    publish_ms_sum = 0.0;
                }
            }
        });

        std::printf("[Run] preset=%s hz=%.1f port=%d buttons=%s vis=%s viewer=%dx%d spine=%s offset_to_ground=%s\n",
            cfg.preset.c_str(),
            cfg.publish_hz,
            cfg.xsens_port,
            cfg.require_buttons ? "on" : "off",
            cfg.vis ? "on" : "off",
            cfg.viewer_width,
            cfg.viewer_height,
            spine_on ? (spine_joint + " " + std::to_string(spine_deg) + "deg").c_str() : "off",
            cfg.offset_to_ground ? "on" : "off");

        // ── Main thread: viewer / lifetime loop ───────────────────────────
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
