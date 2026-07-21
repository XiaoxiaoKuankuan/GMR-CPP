/**
 * smplx_server.cpp — original-GMR SMPL-X SMP1 UDP -> GMR IK -> Redis.
 *
 * The same source is built as smplx_g1_server, smplx_e1_server and
 * smplx_bumi3_server with
 * separate compile-time defaults.  The GMR IK core remains untouched.
 */

#include "gmr/frame_queue.hpp"
#include "gmr/gmr_mink.hpp"
#include "gmr/motion_buffer.hpp"
#include "gmr/mujoco_viewer.hpp"
#include "gmr/redis_publisher.hpp"
#include "readers/smplx_reader.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef SMPLX_DEFAULT_PRESET
#define SMPLX_DEFAULT_PRESET "e1"
#endif
#ifndef SMPLX_DEFAULT_PORT
#define SMPLX_DEFAULT_PORT 7005
#endif

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
    int         port          = SMPLX_DEFAULT_PORT;
    std::string xml_file;
    std::string ik_config;
    std::string preset        = SMPLX_DEFAULT_PRESET;
    double      human_height  = 1.8;
    double      damping       = 1.0;
    double      publish_hz    = 50.0;
    double      stale_ms      = 250.0;
    bool        vis           = false;
    bool        vis_smplx_targets = false;
    bool        vis_smplx_frames = false;
    int         viewer_width  = 640;
    int         viewer_height = 480;
    std::string viewer_follow_body = "pelvis";
    bool        offset_to_ground = false;
    double      fixed_ground_offset = 0.0;
    double      ground_clearance = 0.06;
    bool        require_buttons = true;
    bool        redis_enabled = true;
    bool        redis_order_verified = true;

    gmr::MujocoViewer::RobotBodyMap smplx_robot_body_map;
    std::vector<std::string> diagnostics_joint_names;
    std::vector<std::string> diagnostics_body_names;

    gmr::RedisPublisher::Config redis;
};

struct VisualSpec {
    gmr::MujocoViewer::RobotBodyMap bodies;
    std::vector<std::string> joints;
    std::vector<std::string> body_heights;
};

VisualSpec visualSpecForPreset(const std::string& preset) {
    if (preset == "g1") {
        return {
            {
                {"pelvis", "pelvis"}, {"spine3", "torso_link"},
                {"left_hip", "left_hip_roll_link"},
                {"left_knee", "left_knee_link"},
                {"left_foot", "left_toe_link"},
                {"right_hip", "right_hip_roll_link"},
                {"right_knee", "right_knee_link"},
                {"right_foot", "right_toe_link"},
                {"left_shoulder", "left_shoulder_yaw_link"},
                {"left_elbow", "left_elbow_link"},
                {"left_wrist", "left_wrist_yaw_link"},
                {"right_shoulder", "right_shoulder_yaw_link"},
                {"right_elbow", "right_elbow_link"},
                {"right_wrist", "right_wrist_yaw_link"},
            },
            {
                "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
                "left_shoulder_yaw_joint", "left_elbow_joint",
                "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
                "right_shoulder_yaw_joint", "right_elbow_joint",
                "left_knee_joint", "right_knee_joint",
            },
            {"pelvis", "left_toe_link", "right_toe_link"},
        };
    }
    if (preset == "e1") {
        return {
            {
                {"pelvis", "base_link"}, {"spine3", "waist_roll_link"},
                {"left_hip", "l_leg_hip_pitch_link"},
                {"left_knee", "l_leg_knee_link"},
                {"left_foot", "l_leg_ankle_roll_link"},
                {"right_hip", "r_leg_hip_pitch_link"},
                {"right_knee", "r_leg_knee_link"},
                {"right_foot", "r_leg_ankle_roll_link"},
                {"left_shoulder", "l_arm_shoulder_roll_link"},
                {"left_elbow", "l_arm_elbow_pitch_link"},
                {"left_wrist", "l_arm_elbow_yaw_link"},
                {"right_shoulder", "r_arm_shoulder_roll_link"},
                {"right_elbow", "r_arm_elbow_pitch_link"},
                {"right_wrist", "r_arm_elbow_yaw_link"},
            },
            {
                "l_arm_shoulder_pitch_joint", "l_arm_shoulder_roll_joint",
                "l_arm_shoulder_yaw_joint", "l_arm_elbow_pitch_joint",
                "l_arm_elbow_yaw_joint", "r_arm_shoulder_pitch_joint",
                "r_arm_shoulder_roll_joint", "r_arm_shoulder_yaw_joint",
                "r_arm_elbow_pitch_joint", "r_arm_elbow_yaw_joint",
                "l_leg_knee_joint", "r_leg_knee_joint",
            },
            {"base_link", "l_leg_ankle_roll_link", "r_leg_ankle_roll_link"},
        };
    }
    if (preset == "bumi3") {
        return {
            {
                {"pelvis", "base_link"}, {"spine3", "waist_yaw_link"},
                {"left_hip", "l_leg_roll_link"},
                {"left_knee", "l_knee_pitch_link"},
                {"left_foot", "l_ankle_roll_link"},
                {"right_hip", "r_leg_roll_link"},
                {"right_knee", "r_knee_pitch_link"},
                {"right_foot", "r_ankle_roll_link"},
                {"left_shoulder", "l_arm_yaw_link"},
                {"left_elbow", "l_elbow_pitch_link"},
                {"right_shoulder", "r_arm_yaw_link"},
                {"right_elbow", "r_elbow_pitch_link"},
            },
            {
                "waist_yaw_joint", "l_arm_pitch_joint", "l_arm_roll_joint",
                "l_arm_yaw_joint", "l_elbow_pitch_joint", "r_arm_pitch_joint",
                "r_arm_roll_joint", "r_arm_yaw_joint", "r_elbow_pitch_joint",
                "l_leg_pitch_joint", "l_leg_roll_joint", "l_leg_yaw_joint",
                "l_knee_pitch_joint", "l_ankle_pitch_joint", "l_ankle_roll_joint",
                "r_leg_pitch_joint", "r_leg_roll_joint", "r_leg_yaw_joint",
                "r_knee_pitch_joint", "r_ankle_pitch_joint", "r_ankle_roll_joint",
            },
            {"base_link", "l_ankle_roll_link", "r_ankle_roll_link"},
        };
    }
    throw std::runtime_error("SMPL-X server preset must be g1, e1 or bumi3");
}

struct ModelInfo {
    int nq = 0;
    int nv = 0;
    int nu = 0;
    int nbody = 0;
    int actuated_joints = 0;
};

ModelInfo inspectRobotModel(
    const std::string& xml_path,
    const gmr::MujocoViewer::RobotBodyMap& robot_body_map,
    bool require_base_freejoint) {
    char error[1024] = {};
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!model)
        throw std::runtime_error("BUMI3/model XML validation failed: " +
                                 std::string(error));
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> holder(model, mj_deleteModel);

    for (const auto& [human_name, robot_name] : robot_body_map) {
        (void)human_name;
        if (mj_name2id(model, mjOBJ_BODY, robot_name.c_str()) < 0)
            throw std::runtime_error("viewer/config robot body missing from XML: " +
                                     robot_name);
    }
    if (require_base_freejoint) {
        const int base_id = mj_name2id(model, mjOBJ_BODY, "base_link");
        bool found_freejoint = false;
        for (int joint = 0; joint < model->njnt; ++joint) {
            if (model->jnt_bodyid[joint] == base_id &&
                model->jnt_type[joint] == mjJNT_FREE &&
                model->jnt_qposadr[joint] == 0) {
                found_freejoint = true;
            }
        }
        if (base_id < 0 || !found_freejoint)
            throw std::runtime_error(
                "BUMI3 base_link must own the qpos[0] freejoint");
    }

    int actuated_joints = 0;
    for (int joint = 0; joint < model->njnt; ++joint) {
        if (model->jnt_type[joint] == mjJNT_HINGE ||
            model->jnt_type[joint] == mjJNT_SLIDE) {
            ++actuated_joints;
        }
    }
    return {model->nq, model->nv, model->nu, model->nbody, actuated_joints};
}

void usage(const char* program) {
    std::printf(
        "Usage: %s [options]\n"
        "  --bind <IPv4>             UDP bind address (default 0.0.0.0)\n"
        "  --port <port>             SMP1 UDP port (G1 7004, E1 7005, BUMI3 7006)\n"
        "  --xml <path>              MuJoCo robot XML\n"
        "  --ik-config <path>        SMPL-X-to-robot IK JSON\n"
        "  --human-height <m>        Height used by GMR scaling (default 1.8)\n"
        "  --damping <value>         IK damping (default 1.0)\n"
        "  --redis-host <host>       default 127.0.0.1\n"
        "  --redis-port <port>       default 6379\n"
        "  --redis-db <db>           default 0\n"
        "  --redis-key <key>         override SMPL-X preset key\n"
        "  --hz <rate>               Redis publish rate (default 50)\n"
        "  --ttl-ms <ms>             Redis current-key TTL (default 200)\n"
        "  --stale-ms <ms>           stop publishing after input loss (default 250)\n"
        "  --lin-vel-alpha <0..1>    root linear-velocity EMA\n"
        "  --ang-vel-alpha <0..1>    root angular-velocity EMA\n"
        "  --lin-vel-max <m/s>       reject linear-velocity spikes\n"
        "  --ang-vel-max <rad/s>     reject angular-velocity spikes\n"
        "  --pelvis-z-offset <m>     output-only pelvis z offset\n"
        "  --no-redis                disable Redis connection/publisher\n"
        "  --redis                   explicitly enable Redis publisher\n"
        "  --offset-to-ground        per-frame lowest-foot grounding\n"
        "  --no-offset-to-ground     preserve sender ground (default)\n"
        "  --fixed-ground-offset <m> fixed Z amount subtracted from every target\n"
        "  --ground-clearance <m>    fixed lowest-foot clearance (G1/E1 0.06, BUMI3 0.02)\n"
        "  --always                  bypass A+R1 joystick publish gate\n"
        "  --vis                     open MuJoCo viewer\n"
        "  --vis-smplx-targets       show raw/scaled/robot target overlay\n"
        "  --vis-smplx-frames        add read-only SMPL-X frame axes/console diagnostics\n"
        "  --viewer-width <px>       default 640\n"
        "  --viewer-height <px>      default 480\n"
        "  --viewer-follow-body <b>  override camera follow body\n"
        "  --help\n",
        program);
}

Config parseArgs(int argc, char** argv) {
    Config cfg;
    namespace fs = std::filesystem;
    const fs::path executable_dir = fs::canonical(argv[0]).parent_path();
    const fs::path repo_root = executable_dir.parent_path();
    if (cfg.preset == "g1") {
        cfg.xml_file =
            (repo_root / "assets/unitree_g1/g1_mocap_29dof.xml").string();
        cfg.ik_config =
            (repo_root / "config/ik_configs/smplx_to_g1.json").string();
        cfg.viewer_follow_body = "pelvis";
    } else if (cfg.preset == "e1") {
        cfg.xml_file = (repo_root / "assets/e1/mjcf/e1_24dof.xml").string();
        cfg.ik_config =
            (repo_root / "config/ik_configs/smplx_to_e1.json").string();
        cfg.viewer_follow_body = "base_link";
    } else if (cfg.preset == "bumi3") {
        cfg.xml_file = (repo_root / "assets/bumi3/mjcf/bumi3.xml").string();
        cfg.ik_config =
            (repo_root / "config/ik_configs/smplx_to_bumi3.json").string();
        cfg.viewer_follow_body = "base_link";
        cfg.ground_clearance = 0.02;
        cfg.redis_order_verified = true;
    } else {
        throw std::runtime_error("SMPL-X server preset must be g1, e1 or bumi3");
    }

    const VisualSpec visual = visualSpecForPreset(cfg.preset);
    cfg.smplx_robot_body_map = visual.bodies;
    cfg.diagnostics_joint_names = visual.joints;
    cfg.diagnostics_body_names = visual.body_heights;

    bool explicit_key = false;
    std::string requested_key;
    double pelvis_z_offset = cfg.preset == "g1" ? -0.05 : 0.0;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (++i >= argc)
                throw std::runtime_error("missing value for " + arg);
            return argv[i];
        };

        if      (arg == "--bind")              cfg.bind_ip = next();
        else if (arg == "--port")              cfg.port = std::stoi(next());
        else if (arg == "--xml")               cfg.xml_file = next();
        else if (arg == "--ik-config")         cfg.ik_config = next();
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
        else if (arg == "--no-redis")          cfg.redis_enabled = false;
        else if (arg == "--redis")             cfg.redis_enabled = true;
        else if (arg == "--offset-to-ground")  cfg.offset_to_ground = true;
        else if (arg == "--no-offset-to-ground") cfg.offset_to_ground = false;
        else if (arg == "--fixed-ground-offset") cfg.fixed_ground_offset = std::stod(next());
        else if (arg == "--ground-clearance")  cfg.ground_clearance = std::stod(next());
        else if (arg == "--always")            cfg.require_buttons = false;
        else if (arg == "--vis")               cfg.vis = true;
        else if (arg == "--vis-smplx-targets")  {
            cfg.vis_smplx_targets = true;
            cfg.vis = true;
        }
        else if (arg == "--vis-smplx-frames")  {
            cfg.vis_smplx_frames = true;
            cfg.vis_smplx_targets = true;
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

    if (cfg.preset == "g1") {
        cfg.redis.applyPreset(gmr::presetG1());
        if (!explicit_key) cfg.redis.key = "smplx_online_frame_g1";
    } else if (cfg.preset == "e1") {
        cfg.redis.applyPreset(gmr::presetE1());
        if (!explicit_key) cfg.redis.key = "smplx_online_frame_e1";
    } else if (cfg.preset == "bumi3") {
        cfg.redis.applyPreset(gmr::presetBumi3Gmt());
        if (!explicit_key) cfg.redis.key = "gmt_online_frame_bumi";
    } else {
        throw std::runtime_error("SMPL-X server preset must be g1, e1 or bumi3");
    }

    if (explicit_key) cfg.redis.key = requested_key;
    cfg.redis.pelvis_z_offset = pelvis_z_offset;

    if (cfg.publish_hz <= 0.0) throw std::runtime_error("--hz must be > 0");
    if (cfg.stale_ms <= 0.0)   throw std::runtime_error("--stale-ms must be > 0");
    if (!std::isfinite(cfg.fixed_ground_offset))
        throw std::runtime_error("--fixed-ground-offset must be finite");
    if (!std::isfinite(cfg.ground_clearance) || cfg.ground_clearance < 0.0)
        throw std::runtime_error("--ground-clearance must be finite and >= 0");
    if (!fs::is_regular_file(cfg.xml_file))
        throw std::runtime_error("robot XML not found: " + cfg.xml_file);
    if (!fs::is_regular_file(cfg.ik_config))
        throw std::runtime_error("SMPL-X IK config not found: " + cfg.ik_config);
    if (cfg.redis_enabled && cfg.redis.ttl_ms <= 0)
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
        const ModelInfo model_info = inspectRobotModel(
            cfg.xml_file, cfg.smplx_robot_body_map, cfg.preset == "bumi3");
        const int joints = model_info.actuated_joints;
        const int frame_floats = 14 + joints;
        std::printf(
            "[Config] protocol=SMP1 preset=%s\n"
            "[Config] xml=%s\n"
            "[Config] ik=%s\n"
            "[Config] nq=%d nv=%d nu=%d nbody=%d actuated_joints=%d targets=%zu\n"
            "[Config] redis=%s key=%s order_verified=%s\n"
            "[Config] frame=%d floats / %d bytes\n"
            "[Config] bind=%s:%d stale_ms=%.0f "
            "offset_to_ground=%s fixed_ground_offset=%.3f "
            "ground_clearance=%.3f "
            "vis_smplx_targets=%s vis_smplx_frames=%s\n",
            cfg.preset.c_str(), cfg.xml_file.c_str(), cfg.ik_config.c_str(),
            model_info.nq, model_info.nv, model_info.nu, model_info.nbody,
            model_info.actuated_joints, cfg.smplx_robot_body_map.size(),
            cfg.redis_enabled ? "enabled" : "disabled", cfg.redis.key.c_str(),
            cfg.redis_order_verified ? "yes" : "no",
            frame_floats, frame_floats * 4,
            cfg.bind_ip.c_str(), cfg.port, cfg.stale_ms,
            cfg.offset_to_ground ? "on" : "off",
            cfg.fixed_ground_offset,
            cfg.ground_clearance,
            cfg.vis_smplx_targets ? "on" : "off",
            cfg.vis_smplx_frames ? "on" : "off");

        gmr::FrameQueue queue(300);

        gmr::SmplxReader::Config reader_cfg;
        reader_cfg.bind_ip = cfg.bind_ip;
        reader_cfg.port = cfg.port;
        reader_cfg.verbose = true;
        gmr::SmplxReader reader(queue, reader_cfg);
        reader.connect();

        gmr_mink::GMR gmr(
            cfg.xml_file, cfg.ik_config, cfg.human_height, cfg.damping, false);
        gmr.setGroundOffset(cfg.fixed_ground_offset);
        gmr.setGroundClearance(cfg.ground_clearance);
        std::cout << "[GMR] ready; waiting for SMP1 frames...\n";

        constexpr int    kSeedFrames = 10;
        constexpr double kFrameTimeoutSec = 0.02;
        constexpr double kMaxBufferSec = 12.0;
        const size_t max_frames =
            std::max<size_t>(static_cast<size_t>(
                kMaxBufferSec / kFrameTimeoutSec), 32);

        gmr::MotionBuffer buffer(max_frames, kFrameTimeoutSec);
        buffer.setOffsetToGround(cfg.offset_to_ground);
        buffer.setCaptureTargetData(cfg.vis_smplx_targets);

        std::cout << "[Init] collecting " << kSeedFrames << " seed frames...\n";
        buffer.seedSync(kSeedFrames, queue, &gmr);

        // Re-warm with a real SMPL-X pose while preserving the solver's warm state.
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
                cfg.viewer_follow_body, cfg.smplx_robot_body_map,
                cfg.diagnostics_joint_names, cfg.diagnostics_body_names);

        const auto period =
            std::chrono::duration<double>(1.0 / cfg.publish_hz);

        std::thread publisher_thread;
        if (cfg.redis_enabled) {
            publisher_thread = std::thread([&] {
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
                                "[Safety] SMP1 input stale (%.1f ms); Redis refresh paused. "
                                "The current key will expire via TTL.\n",
                                input_age_ms);
                            last_stale_warning = warning_now;
                        }
                    } else {
                        auto frame = buffer.latestProcessedFrame();
                        if (frame && frame->qpos.size() > 0) {
                            // frame_time is SmplxReader's local steady receive time.
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
                            "[SmplxPubDiag] tick=%.1fHz sent=%.1fHz input=%.1fms "
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
        } else if (!cfg.redis_order_verified) {
            std::cout << "[Safety] Redis disabled: BUMI3 downstream joint publish "
                         "order is unverified. Viewer/IK remain active.\n";
        }
        if (cfg.redis_enabled && !cfg.redis_order_verified) {
            std::cerr << "[Safety] Redis explicitly enabled with UNVERIFIED BUMI3 "
                         "identity qpos order; viewer/test only, not approved for hardware.\n";
        }

        std::printf(
            "[Run] SMP1 UDP=%s:%d preset=%s Redis=%s key=%s "
            "publish=%.1fHz stale=%.0fms ttl=%dms offset_to_ground=%s "
            "fixed_ground_offset=%.3f ground_clearance=%.3f "
            "gate=%s vis=%s targets=%s frames=%s\n",
            cfg.bind_ip.c_str(), cfg.port, cfg.preset.c_str(),
            cfg.redis_enabled ? "on" : "off",
            cfg.redis.key.c_str(), cfg.publish_hz, cfg.stale_ms,
            cfg.redis.ttl_ms,
            cfg.offset_to_ground ? "on" : "off",
            cfg.fixed_ground_offset,
            cfg.ground_clearance,
            cfg.require_buttons ? "A+R1" : "off",
            cfg.vis ? "on" : "off",
            cfg.vis_smplx_targets ? "on" : "off",
            cfg.vis_smplx_frames ? "on" : "off");

        while (!g_stop) {
            const auto t0 = std::chrono::steady_clock::now();
            if (viewer) {
                auto frame = buffer.latestProcessedFrame();
                if (frame && frame->qpos.size() > 0) {
                    if (cfg.vis_smplx_targets) {
                        viewer->render(
                            frame->qpos, &frame->body_data, &frame->target_data,
                            cfg.vis_smplx_frames);
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
