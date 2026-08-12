#include "adapters/g1_motion_adapter.hpp"
#include "gmr/geometry_ground.hpp"
#include "gmr/foot_contact_json.hpp"
#include "gmr/gmr_mink.hpp"
#include "gmr/npy_io.hpp"
#include "readers/g1_motion_reader.hpp"

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string input;
    std::string config;
    std::string source_xml;
    std::string target_xml;
    std::string output = "bumi3_motion.npy";
    double fps = 30.0;
    int foot_contact_override = 0;
    double foot_contact_weight_scale = 0.5;
};

Options parseArgs(int argc, char** argv) {
    Options options;
    const std::filesystem::path executable =
        std::filesystem::weakly_canonical(argv[0]);
    const std::filesystem::path root = executable.parent_path().parent_path();
    options.source_xml = (root / "assets/unitree_g1/g1_mocap_29dof.xml").string();
    options.target_xml = (root / "assets/bumi3/mjcf/bumi3.xml").string();
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        auto next = [&]() {
            if (++index >= argc) throw std::runtime_error("missing value for " + arg);
            return std::string(argv[index]);
        };
        if (arg == "--input") options.input = next();
        else if (arg == "--config") options.config = next();
        else if (arg == "--source-xml") options.source_xml = next();
        else if (arg == "--target-xml") options.target_xml = next();
        else if (arg == "--output") options.output = next();
        else if (arg == "--fps") options.fps = std::stod(next());
        else if (arg == "--foot-contact-constraints")
            options.foot_contact_override = 1;
        else if (arg == "--no-foot-contact-constraints")
            options.foot_contact_override = 0;
        else if (arg == "--foot-contact-weight-scale")
            options.foot_contact_weight_scale = std::stod(next());
        else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: " << argv[0]
                << " --input walk.npy --config config/ik_configs/g1_to_bumi3.json\n"
                   "       [--fps 30] [--output bumi3_motion.npy]"
                   " [--source-xml g1.xml] [--target-xml bumi3.xml]"
                   " [--foot-contact-constraints|--no-foot-contact-constraints]"
                   " [--foot-contact-weight-scale 0.5]\n";
            std::exit(0);
        } else throw std::runtime_error("unknown argument: " + arg);
    }
    if (options.input.empty() || options.config.empty())
        throw std::runtime_error("--input and --config are required");
    if (!std::isfinite(options.foot_contact_weight_scale) ||
        options.foot_contact_weight_scale < 0.0 ||
        options.foot_contact_weight_scale > 5.0)
        throw std::runtime_error(
            "--foot-contact-weight-scale must be finite and in [0, 5]");
    return options;
}

nlohmann::json readJson(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open JSON: " + path);
    nlohmann::json value;
    input >> value;
    return value;
}

struct ModelDeleter { void operator()(mjModel* value) const { mj_deleteModel(value); } };
struct DataDeleter { void operator()(mjData* value) const { mj_deleteData(value); } };

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseArgs(argc, argv);
        const nlohmann::json config = readJson(options.config);
        if (config.at("source").at("robot") != "unitree_g1" ||
            config.at("target").at("robot") != "bumi3")
            throw std::runtime_error("config source/target must be unitree_g1/bumi3");
        const auto& root = config.at("root");
        const bool ground_alignment = root.at("ground_alignment").get<bool>();
        const double height_offset = root.at("height_offset").get<double>();
        const std::vector<std::string> target_feet =
            root.at("target_foot_bodies").get<std::vector<std::string>>();

        gmr::G1MotionReader reader(options.fps);
        if (!reader.load_npy(options.input))
            throw std::runtime_error(reader.last_error());
        gmr::G1MotionAdapter adapter(options.source_xml);
        gmr_mink::GMR solver(options.target_xml, options.config, 1.0, 1.0, false);
        const bool foot_contact_enabled = options.foot_contact_override != 0;
        solver.setFootContactEnabled(foot_contact_enabled);
        solver.setFootContactWeightScale(options.foot_contact_weight_scale);
        solver.setMotionPreservingEnabled(foot_contact_enabled);
        solver.setReferenceTimestep(1.0 / options.fps);
        const bool contact_metadata_available = config.contains("foot_contact");
        std::unique_ptr<gmr::FootContactDetector> contact_detector;
        gmr::FootSoleDefinition source_left_sole;
        gmr::FootSoleDefinition source_right_sole;
        gmr::FootSoleDefinition target_left_sole;
        gmr::FootSoleDefinition target_right_sole;
        if (contact_metadata_available) {
            const auto& foot_contact = config.at("foot_contact");
            source_left_sole = gmr::footSoleDefinitionFromJson(
                foot_contact.at("source").at("left"),
                "foot_contact.source.left");
            source_right_sole = gmr::footSoleDefinitionFromJson(
                foot_contact.at("source").at("right"),
                "foot_contact.source.right");
            target_left_sole = gmr::footSoleDefinitionFromJson(
                foot_contact.at("target").at("left"),
                "foot_contact.target.left");
            target_right_sole = gmr::footSoleDefinitionFromJson(
                foot_contact.at("target").at("right"),
                "foot_contact.target.right");
            contact_detector = std::make_unique<gmr::FootContactDetector>(
                gmr::footDetectorConfigFromJson(foot_contact));
        }
        gmr::TargetGroundAligner ground(options.target_xml, target_feet);

        char error[1024] = {};
        std::unique_ptr<mjModel, ModelDeleter> target_model(
            mj_loadXML(options.target_xml.c_str(), nullptr, error, sizeof(error)));
        if (!target_model) throw std::runtime_error(error);
        std::unique_ptr<mjData, DataDeleter> target_data(mj_makeData(target_model.get()));
        if (!target_data) throw std::runtime_error("mj_makeData failed");
        const int left_foot = mj_name2id(
            target_model.get(), mjOBJ_BODY, target_feet.at(0).c_str());
        const int right_foot = mj_name2id(
            target_model.get(), mjOBJ_BODY, target_feet.at(1).c_str());
        if (left_foot < 0 || right_foot < 0)
            throw std::runtime_error("target foot body missing");

        std::vector<double> motion;
        std::vector<double> roots;
        std::vector<double> feet;
        gmr::G1MotionFrame frame;
        size_t frame_index = 0;
        double minimum_sole = std::numeric_limits<double>::infinity();
        double maximum_sole = -std::numeric_limits<double>::infinity();
        double maximum_stance_slip = 0.0;
        double maximum_stance_tilt = 0.0;
        double maximum_stance_corner_height = 0.0;
        double maximum_steady_slip = 0.0;
        double maximum_steady_tilt = 0.0;
        double maximum_steady_corner_height = 0.0;
        double minimum_corner_height = std::numeric_limits<double>::infinity();
        size_t relaxed_contact_frames = 0;
        size_t detected_double_support_frames = 0;
        size_t detected_single_support_frames = 0;
        size_t forced_grounding_frames = 0;
        size_t worst_slip_frame = 0;
        std::string worst_slip_side = "none";
        bool worst_slip_forced = false;
        double worst_slip_blend = 0.0;
        gmr::FootContactState contact_state;
        struct MetricAnchor {
            bool active = false;
            int frames = 0;
            Eigen::Vector2d xy = Eigen::Vector2d::Zero();
        };
        MetricAnchor legacy_left_anchor;
        MetricAnchor legacy_right_anchor;
        double legacy_max_slip = 0.0;
        double legacy_max_tilt = 0.0;
        double legacy_max_corner_height = 0.0;
        double legacy_min_corner_height = std::numeric_limits<double>::infinity();
        while (reader.get_next_frame(frame)) {
            const gmr::BodyMap source_bodies = adapter.to_body_map(frame);
            if (contact_detector) {
                contact_state = contact_detector->update(
                    gmr::observeFoot(adapter.model(), adapter.data(),
                                     gmr::FootSide::Left, source_left_sole),
                    gmr::observeFoot(adapter.model(), adapter.data(),
                                     gmr::FootSide::Right, source_right_sole),
                    frame.timestamp);
                if (contact_state.left_forced || contact_state.right_forced)
                    ++forced_grounding_frames;
                else if (contact_state.left_stance && contact_state.right_stance)
                    ++detected_double_support_frames;
                else if (contact_state.left_stance || contact_state.right_stance)
                    ++detected_single_support_frames;
            }
            Eigen::VectorXd qpos;
            if (frame_index == 0) {
                for (int iteration = 0; iteration < 100; ++iteration)
                    qpos = solver.retarget(source_bodies, false);
                if (ground_alignment) ground.align(qpos, height_offset);
                if (foot_contact_enabled) {
                    solver.setConfiguration(qpos);
                    solver.initializeFootContacts(contact_state);
                    for (int iteration = 0; iteration < 10; ++iteration)
                        qpos = solver.retarget(source_bodies, false);
                }
            } else {
                if (foot_contact_enabled) solver.setFootContactState(contact_state);
                qpos = solver.retarget(source_bodies, false);
                if (!foot_contact_enabled && ground_alignment)
                    ground.align(qpos, height_offset);
            }
            if (!qpos.allFinite())
                throw std::runtime_error("non-finite target qpos at frame " +
                                         std::to_string(frame_index));
            for (int index = 0; index < qpos.size(); ++index)
                motion.push_back(qpos[index]);
            for (int index = 0; index < 7; ++index) roots.push_back(qpos[index]);

            for (int index = 0; index < target_model->nq; ++index)
                target_data->qpos[index] = qpos[index];
            mj_forward(target_model.get(), target_data.get());
            if (!foot_contact_enabled && contact_detector) {
                auto accumulate_legacy = [&](
                        bool stance, MetricAnchor& anchor,
                        const gmr::FootSoleDefinition& definition,
                        gmr::FootSide side) {
                    const auto observation = gmr::observeFoot(
                        target_model.get(), target_data.get(), side, definition);
                    if (!stance) {
                        anchor = {};
                        return;
                    }
                    if (!anchor.active) {
                        anchor.active = true;
                        anchor.frames = 0;
                        anchor.xy = observation.center_world.head<2>();
                    }
                    ++anchor.frames;
                    if (anchor.frames < 5) return;
                    legacy_max_slip = std::max(
                        legacy_max_slip,
                        (observation.center_world.head<2>() - anchor.xy).norm());
                    legacy_max_tilt = std::max(
                        legacy_max_tilt,
                        std::acos(std::clamp(
                            observation.normal_world.dot(Eigen::Vector3d::UnitZ()),
                            -1.0, 1.0)) * 180.0 / std::acos(-1.0));
                    for (const auto& point : observation.points_world) {
                        legacy_max_corner_height = std::max(
                            legacy_max_corner_height, point.z());
                        legacy_min_corner_height = std::min(
                            legacy_min_corner_height, point.z());
                    }
                };
                accumulate_legacy(
                    contact_state.left_stance, legacy_left_anchor,
                    target_left_sole, gmr::FootSide::Left);
                accumulate_legacy(
                    contact_state.right_stance, legacy_right_anchor,
                    target_right_sole, gmr::FootSide::Right);
            }
            for (const int body : {left_foot, right_foot}) {
                feet.push_back(target_data->xpos[3 * body]);
                feet.push_back(target_data->xpos[3 * body + 1]);
                feet.push_back(target_data->xpos[3 * body + 2]);
            }
            const double sole = ground.lowest(qpos);
            minimum_sole = std::min(minimum_sole, sole);
            maximum_sole = std::max(maximum_sole, sole);
            if (foot_contact_enabled) {
                const auto& diagnostics = solver.footContactDiagnostics();
                if (diagnostics.left_stance) {
                    if (diagnostics.left_slip > maximum_stance_slip) {
                        worst_slip_frame = frame_index;
                        worst_slip_side = "left";
                        worst_slip_forced = diagnostics.left_forced;
                        worst_slip_blend = diagnostics.left_blend;
                    }
                    maximum_stance_slip = std::max(
                        maximum_stance_slip, diagnostics.left_slip);
                    maximum_stance_tilt = std::max(
                        maximum_stance_tilt, diagnostics.left_tilt_degrees);
                    maximum_stance_corner_height = std::max(
                        maximum_stance_corner_height,
                        diagnostics.left_max_height);
                    if (diagnostics.left_blend >= 0.999) {
                        maximum_steady_slip = std::max(
                            maximum_steady_slip, diagnostics.left_slip);
                        maximum_steady_tilt = std::max(
                            maximum_steady_tilt,
                            diagnostics.left_tilt_degrees);
                        maximum_steady_corner_height = std::max(
                            maximum_steady_corner_height,
                            diagnostics.left_max_height);
                    }
                }
                if (diagnostics.right_stance) {
                    if (diagnostics.right_slip > maximum_stance_slip) {
                        worst_slip_frame = frame_index;
                        worst_slip_side = "right";
                        worst_slip_forced = diagnostics.right_forced;
                        worst_slip_blend = diagnostics.right_blend;
                    }
                    maximum_stance_slip = std::max(
                        maximum_stance_slip, diagnostics.right_slip);
                    maximum_stance_tilt = std::max(
                        maximum_stance_tilt, diagnostics.right_tilt_degrees);
                    maximum_stance_corner_height = std::max(
                        maximum_stance_corner_height,
                        diagnostics.right_max_height);
                    if (diagnostics.right_blend >= 0.999) {
                        maximum_steady_slip = std::max(
                            maximum_steady_slip, diagnostics.right_slip);
                        maximum_steady_tilt = std::max(
                            maximum_steady_tilt,
                            diagnostics.right_tilt_degrees);
                        maximum_steady_corner_height = std::max(
                            maximum_steady_corner_height,
                            diagnostics.right_max_height);
                    }
                }
                minimum_corner_height = std::min(
                    minimum_corner_height,
                    std::min(diagnostics.left_min_height,
                             diagnostics.right_min_height));
                if (diagnostics.relaxed_slip_constraints)
                    ++relaxed_contact_frames;
            }
            ++frame_index;
        }
        if (frame_index == 0) throw std::runtime_error("input motion is empty");

        const std::filesystem::path motion_path = options.output;
        const std::filesystem::path directory = motion_path.has_parent_path()
            ? motion_path.parent_path() : std::filesystem::path(".");
        std::filesystem::create_directories(directory);
        const std::string stem = motion_path.stem().string();
        const std::filesystem::path root_path = directory / (stem + "_root.npy");
        const std::filesystem::path foot_path = directory / (stem + "_feet.npy");
        const std::filesystem::path replay_path = directory / (stem + "_replay.json");
        gmr::npy::save2D(motion_path.string(), motion, frame_index,
                         static_cast<size_t>(target_model->nq));
        gmr::npy::save2D(root_path.string(), roots, frame_index, 7);
        gmr::npy::save2D(foot_path.string(), feet, frame_index, 6);

        nlohmann::json replay = {
            {"format", "gmr_mujoco_qpos_replay_v1"},
            {"robot", "bumi3"}, {"xml", options.target_xml},
            {"qpos", std::filesystem::absolute(motion_path).string()},
            {"root_trajectory", std::filesystem::absolute(root_path).string()},
            {"foot_trajectory", std::filesystem::absolute(foot_path).string()},
            {"fps", options.fps}, {"frames", frame_index},
            {"nq", target_model->nq},
            {"ground_alignment", ground_alignment},
            {"height_offset", height_offset},
            {"foot_contact_constraints", foot_contact_enabled},
            {"foot_contact_weight_scale", options.foot_contact_weight_scale},
            {"motion_preserving_retarget", foot_contact_enabled},
            {"root_z_policy", foot_contact_enabled
                ? "primary_support_fk" : "legacy_global_lowest"},
        };
        if (foot_contact_enabled) {
            replay["foot_contact_metrics"] = {
                {"max_stance_slip_m", maximum_stance_slip},
                {"max_stance_tilt_degrees", maximum_stance_tilt},
                {"max_stance_corner_height_m", maximum_stance_corner_height},
                {"steady_max_slip_m", maximum_steady_slip},
                {"steady_max_tilt_degrees", maximum_steady_tilt},
                {"steady_max_corner_height_m", maximum_steady_corner_height},
                {"minimum_corner_height_m", minimum_corner_height},
                {"relaxed_frames", relaxed_contact_frames},
                {"detected_double_support_frames",
                 detected_double_support_frames},
                {"detected_single_support_frames",
                 detected_single_support_frames},
                {"forced_grounding_frames", forced_grounding_frames},
            };
        }
        std::ofstream replay_file(replay_path);
        replay_file << std::setw(2) << replay << '\n';
        if (!replay_file) throw std::runtime_error("failed writing replay manifest");

        std::cout << "[offline] input=" << options.input << " frames=" << frame_index
                  << " fps=" << options.fps << "\n"
                  << "[offline] qpos=" << motion_path << " shape=(" << frame_index
                  << ',' << target_model->nq << ")\n"
                  << "[offline] root=" << root_path << " shape=(" << frame_index
                  << ",7)\n"
                  << "[offline] feet=" << foot_path << " shape=(" << frame_index
                  << ",6)\n"
                  << "[offline] replay=" << replay_path << "\n"
                  << "[offline] sole_z_range=[" << minimum_sole << ','
                  << maximum_sole << "] height_offset=" << height_offset << "\n";
        if (foot_contact_enabled) {
            std::cout << "[offline] foot_contact=on max_stance_slip="
                      << maximum_stance_slip << "m max_stance_tilt="
                      << maximum_stance_tilt << "deg max_stance_corner_height="
                      << maximum_stance_corner_height
                      << "m min_corner_height=" << minimum_corner_height
                      << "m steady_slip=" << maximum_steady_slip
                      << "m steady_tilt=" << maximum_steady_tilt
                      << "deg steady_corner_height="
                      << maximum_steady_corner_height
                      << "m worst_slip_frame=" << worst_slip_frame
                      << " side=" << worst_slip_side
                      << " forced=" << (worst_slip_forced ? "true" : "false")
                      << " blend=" << worst_slip_blend
                      << " relaxed_frames=" << relaxed_contact_frames
                      << " support_frames=(double="
                      << detected_double_support_frames << ",single="
                      << detected_single_support_frames << ",forced="
                      << forced_grounding_frames << ")\n";
        } else {
            std::cout << "[offline] foot_contact=off legacy_path=true"
                      << " detected_stance_slip=" << legacy_max_slip
                      << "m detected_stance_tilt=" << legacy_max_tilt
                      << "deg detected_corner_height=["
                      << legacy_min_corner_height << ','
                      << legacy_max_corner_height << "]m\n";
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_g1_to_bumi3: " << error.what() << '\n';
        return 1;
    }
}
