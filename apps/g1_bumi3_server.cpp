/**
 * OMG Unitree G1 qpos36 (Redis JSON) -> G1 FK BodyMap -> unchanged GMR IK
 * -> BUMI3 qpos -> optional GMT Redis output.
 */

#include "adapters/g1_motion_adapter.hpp"
#include "gmr/geometry_ground.hpp"
#include "gmr/foot_contact_json.hpp"
#include "gmr/gmr_mink.hpp"
#include "gmr/mujoco_viewer.hpp"
#include "gmr/qpos_interpolator.hpp"
#include "gmr/redis_publisher.hpp"
#include "readers/g1_redis_receiver.hpp"

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

std::atomic<bool> g_stop{false};

namespace {

void signalHandler(int) { g_stop = true; }

struct Config {
    std::string source_xml;
    std::string target_xml;
    std::string ik_config;
    std::string redis_host = "127.0.0.1";
    int redis_port = 6379;
    int redis_db = 0;
    std::string input_key = "omg_online_frame_g1";
    std::string output_key = "gmt_online_frame_bumi";
    double receiver_poll_hz = 60.0;
    double publish_hz = 50.0;
    double stale_ms = 250.0;
    int ttl_ms = 200;
    double damping = 1.0;
    double viewer_ground_penetration = 0.005;
    bool output_redis = true;
    bool vis = false;
    bool vis_targets = false;
    int viewer_width = 640;
    int viewer_height = 480;
    // Compatibility is deliberate: no flag means the pre-contact legacy path.
    int foot_contact_override = 0;
    double foot_contact_weight_scale = 0.5;
    int reference_interpolation_override = -1;
};

nlohmann::json readJson(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open JSON: " + path);
    nlohmann::json value;
    input >> value;
    return value;
}

void usage(const char* program) {
    std::cout
        << "Usage: " << program << " [options]\n"
        << "  --config <json>             g1_to_bumi3.json\n"
        << "  --source-xml <xml>          Unitree G1 29-DoF MuJoCo model\n"
        << "  --target-xml <xml>          BUMI3 MuJoCo model\n"
        << "  --redis-host <host>         input/output Redis host (127.0.0.1)\n"
        << "  --redis-port <port>         input/output Redis port (6379)\n"
        << "  --redis-db <db>             input/output Redis DB (0)\n"
        << "  --redis-key <key>           OMG input key alias\n"
        << "  --input-redis-key <key>     OMG input key (omg_online_frame_g1)\n"
        << "  --output-redis-key <key>    GMT output key (gmt_online_frame_bumi)\n"
        << "  --poll-hz <rate>            input Redis polling rate (60)\n"
        << "  --hz <rate>                 BUMI3 output publish rate (50)\n"
        << "  --ttl-ms <ms>               BUMI3 output key TTL (200)\n"
        << "  --stale-ms <ms>             stop output after source loss (250)\n"
        << "  --damping <value>           existing GMR constructor damping (1.0)\n"
        << "  --viewer-ground-penetration <m>  viewer-only foot seating (0.005)\n"
        << "  --ground-penetration <m>    compatibility alias for the option above\n"
        << "  --no-output-redis           viewer/offline inspection only\n"
        << "  --vis                       open BUMI3 MuJoCo viewer\n"
        << "  --vis-targets               show G1 FK target markers\n"
        << "  --viewer-width <px>         viewer width (640)\n"
        << "  --viewer-height <px>        viewer height (480)\n";
    std::cout
        << "  --foot-contact-constraints enable gait-aware retarget + soft foot contact\n"
        << "  --no-foot-contact-constraints  preserve legacy IK + per-frame ground align\n"
        << "  --foot-contact-weight-scale <s> scale soft contact weights [0,5] (0.5)\n"
        << "  --reference-interpolation smooth viewer/GMT qpos between source frames\n"
        << "  --no-reference-interpolation  disable reference interpolation\n";
}

Config parseArgs(int argc, char** argv) {
    Config config;
    const std::filesystem::path executable =
        std::filesystem::weakly_canonical(argv[0]);
    const std::filesystem::path root = executable.parent_path().parent_path();
    config.source_xml = (root / "assets/unitree_g1/g1_mocap_29dof.xml").string();
    config.target_xml = (root / "assets/bumi3/mjcf/bumi3.xml").string();
    config.ik_config = (root / "config/ik_configs/g1_to_bumi3.json").string();
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        auto next = [&]() {
            if (++index >= argc) throw std::runtime_error("missing value for " + arg);
            return std::string(argv[index]);
        };
        if (arg == "--config" || arg == "--ik-config") config.ik_config = next();
        else if (arg == "--source-xml") config.source_xml = next();
        else if (arg == "--target-xml" || arg == "--xml") config.target_xml = next();
        else if (arg == "--redis-host") config.redis_host = next();
        else if (arg == "--redis-port") config.redis_port = std::stoi(next());
        else if (arg == "--redis-db") config.redis_db = std::stoi(next());
        else if (arg == "--redis-key" || arg == "--input-redis-key")
            config.input_key = next();
        else if (arg == "--output-redis-key") config.output_key = next();
        else if (arg == "--poll-hz") config.receiver_poll_hz = std::stod(next());
        else if (arg == "--hz") config.publish_hz = std::stod(next());
        else if (arg == "--ttl-ms") config.ttl_ms = std::stoi(next());
        else if (arg == "--stale-ms") config.stale_ms = std::stod(next());
        else if (arg == "--damping") config.damping = std::stod(next());
        else if (arg == "--viewer-ground-penetration" ||
                 arg == "--ground-penetration")
            config.viewer_ground_penetration = std::stod(next());
        else if (arg == "--no-output-redis" || arg == "--no-redis")
            config.output_redis = false;
        else if (arg == "--output-redis" || arg == "--redis")
            config.output_redis = true;
        else if (arg == "--vis") config.vis = true;
        else if (arg == "--vis-targets") {
            config.vis = true;
            config.vis_targets = true;
        } else if (arg == "--viewer-width") config.viewer_width = std::stoi(next());
        else if (arg == "--viewer-height") config.viewer_height = std::stoi(next());
        else if (arg == "--foot-contact-constraints")
            config.foot_contact_override = 1;
        else if (arg == "--no-foot-contact-constraints")
            config.foot_contact_override = 0;
        else if (arg == "--foot-contact-weight-scale")
            config.foot_contact_weight_scale = std::stod(next());
        else if (arg == "--reference-interpolation")
            config.reference_interpolation_override = 1;
        else if (arg == "--no-reference-interpolation")
            config.reference_interpolation_override = 0;
        else if (arg == "--always") { /* compatibility: no joystick gate here */ }
        else if (arg == "--help" || arg == "-h") { usage(argv[0]); std::exit(0); }
        else throw std::runtime_error("unknown argument: " + arg);
    }
    for (const auto& path : {config.source_xml, config.target_xml, config.ik_config})
        if (!std::filesystem::is_regular_file(path))
            throw std::runtime_error("required file not found: " + path);
    if (config.publish_hz <= 0.0 || config.receiver_poll_hz <= 0.0 ||
        config.stale_ms <= 0.0)
        throw std::runtime_error("--hz, --poll-hz and --stale-ms must be > 0");
    if (!std::isfinite(config.viewer_ground_penetration) ||
        config.viewer_ground_penetration < 0.0 ||
        config.viewer_ground_penetration > 0.03)
        throw std::runtime_error(
            "--viewer-ground-penetration must be finite and in [0, 0.03] meters");
    if (!std::isfinite(config.foot_contact_weight_scale) ||
        config.foot_contact_weight_scale < 0.0 ||
        config.foot_contact_weight_scale > 5.0)
        throw std::runtime_error(
            "--foot-contact-weight-scale must be finite and in [0, 5]");
    return config;
}

double steadySeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    try {
        const Config config = parseArgs(argc, argv);
        const nlohmann::json ik = readJson(config.ik_config);
        if (ik.at("source").at("robot") != "unitree_g1" ||
            ik.at("target").at("robot") != "bumi3")
            throw std::runtime_error("IK config source/target must be unitree_g1/bumi3");
        const auto& root_config = ik.at("root");
        const bool ground_alignment =
            root_config.at("ground_alignment").get<bool>();
        const double height_offset = root_config.at("height_offset").get<double>();
        const std::vector<std::string> target_feet =
            root_config.at("target_foot_bodies").get<std::vector<std::string>>();
        std::map<std::string, std::string> viewer_mapping;
        for (const auto& [source, target] : ik.at("body_mapping").items())
            viewer_mapping[source] = target.get<std::string>();

        gmr::G1RedisReceiver::Config receiver_config;
        receiver_config.host = config.redis_host;
        receiver_config.port = config.redis_port;
        receiver_config.db = config.redis_db;
        receiver_config.key = config.input_key;
        receiver_config.poll_hz = config.receiver_poll_hz;
        gmr::G1RedisReceiver receiver(receiver_config);
        receiver.connect();

        gmr::G1MotionAdapter adapter(config.source_xml);
        gmr_mink::GMR solver(config.target_xml, config.ik_config,
                             1.0, config.damping, false);
        const bool foot_contact_enabled = config.foot_contact_override != 0;
        const bool reference_interpolation_enabled =
            config.reference_interpolation_override >= 0
                ? config.reference_interpolation_override != 0
                : foot_contact_enabled;
        solver.setFootContactEnabled(foot_contact_enabled);
        solver.setFootContactWeightScale(config.foot_contact_weight_scale);
        solver.setMotionPreservingEnabled(foot_contact_enabled);
        solver.setReferenceTimestep(1.0 / config.publish_hz);
        gmr::QposInterpolator::Config interpolation_config;
        interpolation_config.default_interval_seconds =
            std::clamp(1.0 / config.publish_hz,
                       interpolation_config.minimum_interval_seconds,
                       interpolation_config.maximum_interval_seconds);
        gmr::QposInterpolator reference_interpolator(interpolation_config);
        std::unique_ptr<gmr::FootContactDetector> contact_detector;
        gmr::FootSoleDefinition source_left_sole;
        gmr::FootSoleDefinition source_right_sole;
        if (foot_contact_enabled) {
            const auto& foot = ik.at("foot_contact");
            source_left_sole = gmr::footSoleDefinitionFromJson(
                foot.at("source").at("left"), "foot_contact.source.left");
            source_right_sole = gmr::footSoleDefinitionFromJson(
                foot.at("source").at("right"), "foot_contact.source.right");
            contact_detector = std::make_unique<gmr::FootContactDetector>(
                gmr::footDetectorConfigFromJson(foot));
        }
        gmr::TargetGroundAligner ground(config.target_xml, target_feet);

        std::unique_ptr<gmr::RedisPublisher> publisher;
        if (config.output_redis) {
            gmr::RedisPublisher::Config output;
            output.host = config.redis_host;
            output.port = config.redis_port;
            output.db = config.redis_db;
            output.key = config.output_key;
            output.ttl_ms = config.ttl_ms;
            output.applyPreset(gmr::presetBumi3Gmt());
            output.key = config.output_key;
            publisher = std::make_unique<gmr::RedisPublisher>(output);
            publisher->clearKey();
        }

        std::unique_ptr<gmr::MujocoViewer> viewer;
        if (config.vis) {
            viewer = std::make_unique<gmr::MujocoViewer>(
                config.target_xml, config.viewer_width, config.viewer_height,
                "base_link", viewer_mapping);
        }

        std::cout << "[Config] OMG Redis=" << config.redis_host << ':'
                  << config.redis_port << '/' << config.redis_db
                  << " key=" << config.input_key << "\n"
                  << "[Config] G1 XML=" << config.source_xml << "\n"
                  << "[Config] BUMI3 XML=" << config.target_xml << "\n"
                  << "[Config] IK=" << config.ik_config << "\n"
                  << "[Config] root ground_alignment="
                  << (ground_alignment ? "on" : "off")
                  << " height_offset=" << height_offset
                  << " viewer_penetration="
                  << config.viewer_ground_penetration
                  << "m (viewer only; GMT stays at exact geometry contact)\n"
                  << "[Config] foot_contact_constraints="
                  << (foot_contact_enabled ? "on" : "off (legacy path)") << "\n"
                  << "[Config] motion_preserving_retarget="
                  << (foot_contact_enabled
                          ? "on (root XY/yaw + torso pitch via hips + primary-support Z)"
                          : "off (legacy full-pose IK)") << "\n"
                  << "[Config] reference_interpolation="
                  << (reference_interpolation_enabled
                          ? "on (one source frame latency)" : "off") << "\n"
                  << "[Config] GMT Redis="
                  << (config.output_redis ? config.output_key : "disabled")
                  << " publish=" << config.publish_hz << "Hz ttl="
                  << config.ttl_ms << "ms stale=" << config.stale_ms << "ms\n"
                  << "[GMR] ready; waiting for OMG G1 frames...\n";
        if (foot_contact_enabled) {
            const auto& constraints =
                ik.at("foot_contact").at("constraints");
            std::cout
                << "[Config] foot_contact_profile="
                << (constraints.value("hard_support_constraints", false)
                        ? "strict" : "soft")
                << " xy_weight=" << constraints.value("xy_weight", 0.0)
                << " runtime_scale=" << config.foot_contact_weight_scale
                << " anchor_deadzone="
                << constraints.value("anchor_deadzone_m", 0.0)
                << "m transition_frames="
                << constraints.value("transition_frames", 0)
                << " temporal_weights=(v="
                << ik.at("motion_preserving").value(
                       "joint_velocity_weight", 0.0)
                << ",a="
                << ik.at("motion_preserving").value(
                       "joint_acceleration_weight", 0.0)
                << ") root_roll_upright_weight="
                << ik.at("motion_preserving").value(
                       "root_roll_upright_weight",
                       ik.at("motion_preserving").value(
                           "root_upright_weight", 0.0))
                << " torso_pitch_weight_scale="
                << ik.at("motion_preserving").value(
                       "torso_pitch_weight_scale", 1.0)
                << " root_z=primary_support_fk\n";
        }

        Eigen::VectorXd latest_qpos;
        Eigen::VectorXd latest_viewer_qpos;
        gmr::BodyMap latest_source;
        gmr::BodyMap latest_scaled;
        double latest_source_timestamp = 0.0;
        double latest_local_time = -std::numeric_limits<double>::infinity();
        bool warmed = false;
        auto next_publish = std::chrono::steady_clock::now();
        const auto publish_period = std::chrono::duration_cast<
            std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / config.publish_hz));
        auto last_stale_warning = std::chrono::steady_clock::now() -
                                  std::chrono::seconds(2);
        gmr::FootContactState latest_contact_state;
        gmr::FootContactState previous_contact_state;
        bool have_contact_state = false;
        auto last_contact_status = std::chrono::steady_clock::now() -
                                   std::chrono::seconds(2);
        double last_publish_reference_timestamp =
            -std::numeric_limits<double>::infinity();
        double previous_source_timestamp =
            std::numeric_limits<double>::quiet_NaN();

        while (!g_stop) {
            gmr::G1MotionFrame frame;
            if (receiver.get_next_frame(frame, 0.001)) {
                if (std::isfinite(previous_source_timestamp) &&
                    frame.timestamp > previous_source_timestamp) {
                    solver.setReferenceTimestep(std::clamp(
                        frame.timestamp - previous_source_timestamp,
                        0.005, 0.1));
                }
                previous_source_timestamp = frame.timestamp;
                latest_source = adapter.to_body_map(frame);
                if (foot_contact_enabled) {
                    const auto left_observation = gmr::observeFoot(
                        adapter.model(), adapter.data(), gmr::FootSide::Left,
                        source_left_sole);
                    const auto right_observation = gmr::observeFoot(
                        adapter.model(), adapter.data(), gmr::FootSide::Right,
                        source_right_sole);
                    latest_contact_state = contact_detector->update(
                        left_observation, right_observation, frame.timestamp);
                    if (!have_contact_state ||
                        latest_contact_state.left_stance !=
                            previous_contact_state.left_stance ||
                        latest_contact_state.right_stance !=
                            previous_contact_state.right_stance ||
                        latest_contact_state.left_forced !=
                            previous_contact_state.left_forced ||
                        latest_contact_state.right_forced !=
                            previous_contact_state.right_forced) {
                        std::cout
                            << "[foot-contact] left="
                            << (latest_contact_state.left_stance ? "stance" : "swing")
                            << (latest_contact_state.left_forced ? "(forced)" : "")
                            << " right="
                            << (latest_contact_state.right_stance ? "stance" : "swing")
                            << (latest_contact_state.right_forced ? "(forced)" : "")
                            << " source_height=("
                            << latest_contact_state.left_height << ','
                            << latest_contact_state.right_height << ")m"
                            << " source_speed_xy=("
                            << latest_contact_state.left_horizontal_speed << ','
                            << latest_contact_state.right_horizontal_speed
                            << ")m/s\n";
                    }
                    previous_contact_state = latest_contact_state;
                    have_contact_state = true;
                }

                if (!warmed) {
                    for (int iteration = 0; iteration < 100; ++iteration)
                        latest_qpos = solver.retarget(latest_source, false);
                    if (ground_alignment) ground.align(latest_qpos, height_offset);
                    if (foot_contact_enabled) {
                        // Ground once, synchronize the solver's internal state,
                        // then capture anchors in the exact configuration that
                        // will be sent to GMT.
                        solver.setConfiguration(latest_qpos);
                        solver.initializeFootContacts(latest_contact_state);
                        for (int iteration = 0; iteration < 10; ++iteration)
                            latest_qpos = solver.retarget(latest_source, false);
                    }
                    warmed = true;
                } else {
                    if (foot_contact_enabled)
                        solver.setFootContactState(latest_contact_state);
                    latest_qpos = solver.retarget(latest_source, false);
                    if (!foot_contact_enabled && ground_alignment)
                        ground.align(latest_qpos, height_offset);
                }
                latest_viewer_qpos = latest_qpos;
                if (viewer && ground_alignment &&
                    config.viewer_ground_penetration > 0.0) {
                    if (foot_contact_enabled)
                        latest_viewer_qpos[2] -=
                            config.viewer_ground_penetration;
                    else
                        ground.align(
                            latest_viewer_qpos,
                            height_offset - config.viewer_ground_penetration);
                }
                latest_scaled = solver.getScaledHumanData();
                latest_source_timestamp = frame.timestamp;
                latest_local_time = steadySeconds();
                if (reference_interpolation_enabled)
                    reference_interpolator.push(
                        latest_qpos, latest_source_timestamp, latest_local_time);

                if (foot_contact_enabled &&
                    std::chrono::steady_clock::now() - last_contact_status >=
                        std::chrono::seconds(1)) {
                    const auto& status = solver.footContactDiagnostics();
                    std::cout
                        << "[foot-contact] target left slip=" << status.left_slip
                        << "m tilt=" << status.left_tilt_degrees
                        << "deg height=[" << status.left_min_height << ','
                        << status.left_max_height << "]m; right slip="
                        << status.right_slip << "m tilt="
                        << status.right_tilt_degrees << "deg height=["
                        << status.right_min_height << ','
                        << status.right_max_height << "]m daqp="
                        << status.daqp_exitflag
                        << " primary="
                        << (status.primary_support_side == 0 ? "left" :
                            status.primary_support_side == 1 ? "right" : "none")
                        << (status.root_z_safety_override
                                ? " root_z_safety" : "")
                        << (status.relaxed_slip_constraints ? " relaxed" : "")
                        << '\n';
                    last_contact_status = std::chrono::steady_clock::now();
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (publisher && now >= next_publish) {
                const double age_ms = (steadySeconds() - latest_local_time) * 1000.0;
                if (latest_qpos.size() > 0 && age_ms <= config.stale_ms) {
                    Eigen::VectorXd publish_qpos = latest_qpos;
                    double publish_timestamp = latest_source_timestamp;
                    if (reference_interpolation_enabled &&
                        reference_interpolator.initialized()) {
                        publish_qpos = reference_interpolator.sample(steadySeconds());
                        if (!foot_contact_enabled && ground_alignment)
                            ground.align(publish_qpos, height_offset);
                        if (std::isfinite(last_publish_reference_timestamp))
                            publish_timestamp = std::max(
                                publish_timestamp,
                                last_publish_reference_timestamp +
                                    1.0 / config.publish_hz);
                        last_publish_reference_timestamp = publish_timestamp;
                    }
                    publisher->publish(publish_qpos, publish_timestamp,
                                       latest_source);
                } else if (now - last_stale_warning >= std::chrono::seconds(1)) {
                    std::cerr << "[Safety] OMG G1 input stale (" << age_ms
                              << " ms); GMT Redis refresh paused, TTL will expire.\n";
                    last_stale_warning = now;
                }
                do { next_publish += publish_period; } while (next_publish <= now);
            }

            if (viewer && latest_qpos.size() > 0) {
                Eigen::VectorXd viewer_qpos = latest_viewer_qpos;
                if (reference_interpolation_enabled &&
                    reference_interpolator.initialized()) {
                    viewer_qpos = reference_interpolator.sample(steadySeconds());
                    if (ground_alignment) {
                        if (foot_contact_enabled)
                            viewer_qpos[2] -=
                                config.viewer_ground_penetration;
                        else
                            ground.align(
                                viewer_qpos,
                                height_offset - config.viewer_ground_penetration);
                    }
                }
                if (config.vis_targets)
                    viewer->render(
                        viewer_qpos,
                        &latest_source, &latest_scaled, false);
                else
                    viewer->render(viewer_qpos);
                if (viewer->shouldClose()) break;
            } else if (!viewer) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        g_stop = true;
        receiver.disconnect();
        std::cout << "[Run] stopped.\n";
        return 0;
    } catch (const std::exception& error) {
        g_stop = true;
        std::cerr << "[fatal] " << error.what() << '\n';
        return 1;
    }
}
