#include "gmr/geometry_ground.hpp"
#include "gmr/foot_contact_json.hpp"

#include <Eigen/Geometry>
#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct ModelDeleter { void operator()(mjModel* value) const { mj_deleteModel(value); } };
struct DataDeleter { void operator()(mjData* value) const { mj_deleteData(value); } };
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

struct Mapping {
    std::string source;
    std::string target;
};

const std::vector<Mapping> kMappings = {
    {"pelvis", "base_link"},
    {"torso_link", "waist_yaw_link"},
    {"left_hip_roll_link", "l_leg_roll_link"},
    {"left_knee_link", "l_knee_pitch_link"},
    {"left_ankle_roll_link", "l_ankle_roll_link"},
    {"right_hip_roll_link", "r_leg_roll_link"},
    {"right_knee_link", "r_knee_pitch_link"},
    {"right_ankle_roll_link", "r_ankle_roll_link"},
    {"left_shoulder_yaw_link", "l_arm_yaw_link"},
    {"left_elbow_link", "l_elbow_pitch_link"},
    {"right_shoulder_yaw_link", "r_arm_yaw_link"},
    {"right_elbow_link", "r_elbow_pitch_link"},
};

constexpr double kG1ToBumi3ElbowZeroOffset = -1.5707963267948966;

bool isElbowMapping(const Mapping& mapping) {
    return mapping.source == "left_elbow_link" ||
           mapping.source == "right_elbow_link";
}

struct Pose {
    Eigen::Vector3d position;
    Eigen::Quaterniond rotation;
};

Pose bodyPose(const mjModel* model, const mjData* data, const std::string& name) {
    const int id = mj_name2id(model, mjOBJ_BODY, name.c_str());
    if (id < 0) throw std::runtime_error("missing body: " + name);
    return {
        Eigen::Vector3d(data->xpos[3 * id], data->xpos[3 * id + 1],
                        data->xpos[3 * id + 2]),
        Eigen::Quaterniond(data->xquat[4 * id], data->xquat[4 * id + 1],
                           data->xquat[4 * id + 2], data->xquat[4 * id + 3]).normalized(),
    };
}

nlohmann::json vectorJson(const Eigen::Vector3d& value) {
    return nlohmann::json::array({value.x(), value.y(), value.z()});
}

nlohmann::json quaternionJson(Eigen::Quaterniond value) {
    value.normalize();
    if (value.w() < 0.0) value.coeffs() *= -1.0;
    return nlohmann::json::array({value.w(), value.x(), value.y(), value.z()});
}

std::pair<ModelPtr, DataPtr> loadModel(const std::string& path) {
    char error[1024] = {};
    ModelPtr model(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
    if (!model) throw std::runtime_error(path + ": " + error);
    DataPtr data(mj_makeData(model.get()));
    if (!data) throw std::runtime_error(path + ": mj_makeData failed");
    mj_forward(model.get(), data.get());
    return {std::move(model), std::move(data)};
}

nlohmann::json readJson(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open reference config: " + path);
    nlohmann::json value;
    input >> value;
    return value;
}

void usage(const char* program) {
    std::cout << "Usage: " << program
              << " --source-xml <g1.xml> --target-xml <bumi3.xml>"
                 " --reference <smplx_to_bumi3.json> --output <g1_to_bumi3.json>\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        std::string source_xml, target_xml, reference_path, output_path;
        for (int index = 1; index < argc; ++index) {
            const std::string arg = argv[index];
            auto next = [&]() {
                if (++index >= argc) throw std::runtime_error("missing value for " + arg);
                return std::string(argv[index]);
            };
            if (arg == "--source-xml") source_xml = next();
            else if (arg == "--target-xml") target_xml = next();
            else if (arg == "--reference") reference_path = next();
            else if (arg == "--output") output_path = next();
            else if (arg == "--help" || arg == "-h") { usage(argv[0]); return 0; }
            else throw std::runtime_error("unknown argument: " + arg);
        }
        if (source_xml.empty() || target_xml.empty() || reference_path.empty() ||
            output_path.empty()) {
            usage(argv[0]);
            return 2;
        }

        auto [source_model, source_data] = loadModel(source_xml);
        auto [target_model, target_data] = loadModel(target_xml);
        if (source_model->nq != 36)
            throw std::runtime_error("OMG G1 source model must have nq=36");
        const nlohmann::json reference = readJson(reference_path);

        const double source_ground = gmr::lowestGeometryZ(
            source_model.get(), source_data.get(),
            {"left_ankle_roll_link", "right_ankle_roll_link"}, true);
        const double target_ground = gmr::lowestGeometryZ(
            target_model.get(), target_data.get(),
            {"l_ankle_roll_link", "r_ankle_roll_link"}, true);
        const Pose source_root = bodyPose(source_model.get(), source_data.get(), "pelvis");
        const Pose target_root = bodyPose(target_model.get(), target_data.get(), "base_link");
        const double source_leg_height = source_root.position.z() - source_ground;
        const double target_leg_height = target_root.position.z() - target_ground;
        if (!(source_leg_height > 0.0) || !(target_leg_height > 0.0))
            throw std::runtime_error("model-derived standing height is not positive");
        const double root_scale = target_leg_height / source_leg_height;
        const gmr::FootSoleDefinition source_left_sole =
            gmr::deriveFootSoleDefinition(
                source_model.get(), source_data.get(), "left_ankle_roll_link");
        const gmr::FootSoleDefinition source_right_sole =
            gmr::deriveFootSoleDefinition(
                source_model.get(), source_data.get(), "right_ankle_roll_link");
        const gmr::FootSoleDefinition target_left_sole =
            gmr::deriveFootSoleDefinition(
                target_model.get(), target_data.get(), "l_ankle_roll_link");
        const gmr::FootSoleDefinition target_right_sole =
            gmr::deriveFootSoleDefinition(
                target_model.get(), target_data.get(), "r_ankle_roll_link");

        std::map<std::string, double> scales;
        std::map<std::string, Eigen::Vector3d> position_offsets;
        std::map<std::string, Eigen::Quaterniond> rotation_offsets;
        for (const Mapping& mapping : kMappings) {
            const Pose source = bodyPose(source_model.get(), source_data.get(), mapping.source);
            const Pose target = bodyPose(target_model.get(), target_data.get(), mapping.target);
            double scale = root_scale;
            if (mapping.source != "pelvis") {
                const double source_radius =
                    (source.position - source_root.position).norm();
                const double target_radius =
                    (target.position - target_root.position).norm();
                if (source_radius < 1e-8 || target_radius < 1e-8)
                    throw std::runtime_error("cannot derive body scale for " + mapping.source);
                scale = target_radius / source_radius;
            }
            scales[mapping.source] = scale;
            Eigen::Vector3d scaled_position;
            if (mapping.source == "pelvis") {
                scaled_position = source.position * root_scale;
            } else {
                scaled_position =
                    (source.position - source_root.position) * scale +
                    source_root.position * root_scale;
            }
            Eigen::Quaterniond rotation_offset =
                source.rotation.conjugate() * target.rotation;
            if (isElbowMapping(mapping)) {
                // The G1 forearm extends along elbow-local +X at q=0, while
                // BUMI3 extends along elbow-local -Z.  Calibrating only the
                // body-frame rest rotations makes the IK request an impossible
                // positive BUMI3 elbow angle and clamp at zero.  A target-local
                // -pi/2 rotation expresses the same physical forearm direction:
                // q_bumi ~= q_g1 - pi/2.
                rotation_offset *= Eigen::Quaterniond(Eigen::AngleAxisd(
                    kG1ToBumi3ElbowZeroOffset, Eigen::Vector3d::UnitY()));
            }
            rotation_offset.normalize();
            const Eigen::Vector3d position_offset =
                target.rotation.conjugate() * (target.position - scaled_position);
            position_offsets[mapping.source] = position_offset;
            rotation_offsets[mapping.source] = rotation_offset;
        }

        nlohmann::json output;
        output["source"] = {
            {"robot", "unitree_g1"},
            {"model", std::filesystem::path(source_xml).lexically_normal().string()},
            {"qpos_layout", "root_pos_xyz + root_quat_wxyz + joint_pos_29"},
        };
        output["target"] = {
            {"robot", "bumi3"},
            {"model", std::filesystem::path(target_xml).lexically_normal().string()},
        };
        output["root"] = {
            {"source", "pelvis"}, {"target", "base_link"},
            {"ground_alignment", true}, {"height_offset", 0.0},
            {"source_foot_bodies", {"left_ankle_roll_link", "right_ankle_roll_link"}},
            {"target_foot_bodies", {"l_ankle_roll_link", "r_ankle_roll_link"}},
            {"source_standing_sole_z", source_ground},
            {"target_standing_sole_z", target_ground},
        };
        output["foot_contact"] = {
            {"enabled", true},
            {"ground_z", 0.0},
            {"allow_flight", false},
            {"source", {
                {"left", gmr::footSoleDefinitionToJson(source_left_sole)},
                {"right", gmr::footSoleDefinitionToJson(source_right_sole)},
            }},
            {"target", {
                {"left", gmr::footSoleDefinitionToJson(target_left_sole)},
                {"right", gmr::footSoleDefinitionToJson(target_right_sole)},
            }},
            {"detection", {
                {"enter_height_m", 0.025},
                {"exit_height_m", 0.045},
                {"max_enter_vertical_speed_mps", 0.18},
                {"max_exit_upward_speed_mps", 0.30},
                {"max_enter_horizontal_speed_mps", 0.035},
                {"max_exit_horizontal_speed_mps", 0.075},
                {"double_support_max_horizontal_speed_mps", 0.025},
                {"enter_frames", 2},
                {"exit_frames", 1},
                {"minimum_stance_frames", 2},
            }},
            {"constraints", {
                {"xy_weight", 35.0},
                {"tilt_weight", 60.0},
                {"height_weight", 80.0},
                {"xy_kp", 5.0},
                {"tilt_kp", 8.0},
                {"height_kp", 10.0},
                {"max_anchor_correction_speed_mps", 0.20},
                {"anchor_deadzone_m", 0.008},
                {"max_slip_speed_mps", 0.05},
                {"support_height_m", 0.001},
                {"support_height_upper_m", 0.003},
                {"penetration_tolerance_m", 0.0},
                {"max_joint_velocity_rps", 8.0},
                {"max_root_linear_velocity_mps", 3.0},
                {"max_root_angular_velocity_rps", 6.0},
                {"forced_support_weight_scale", 0.08},
                {"transition_frames", 8},
                {"hard_support_constraints", false},
            }},
        };
        output["motion_preserving"] = {
            {"enabled", true},
            {"root_body", "base_link"},
            {"waist_body", "waist_yaw_link"},
            // BUMI3 has no waist roll/pitch joints.  Match only horizontal
            // root translation and heading, otherwise the full torso
            // orientation task pushes G1 waist roll/pitch into BUMI3's pelvis.
            {"root_position_axes", {1.0, 1.0, 0.0}},
            {"root_rotation_axes", {0.0, 0.0, 1.0}},
            {"root_roll_upright_weight", 20.0},
            {"torso_pitch_weight_scale", 1.0},
            {"waist_rotation_axes", {0.0, 0.0, 1.0}},
            {"root_z_policy", "primary_support_fk"},
            // These are dimensionless configuration-space terms applied once
            // per input reference, after geometric IK.  They are deliberately
            // light so swing clearance and timing remain source-driven.
            {"joint_velocity_weight", 0.05},
            {"joint_acceleration_weight", 0.20},
        };
        output["body_mapping"] = nlohmann::json::object();
        for (const Mapping& mapping : kMappings)
            output["body_mapping"][mapping.source] = mapping.target;
        output["generated_from"] = {
            {"source_model", source_xml}, {"target_model", target_xml},
            {"weight_reference", reference_path},
            {"method", "rest-FK alignment, model-derived scale, and semantic forearm zero calibration"},
        };
        output["semantic_joint_mapping"] = {
            {"left_elbow_joint", {
                {"target", "l_elbow_pitch_joint"},
                {"scale", 1.0},
                {"offset_radians", kG1ToBumi3ElbowZeroOffset},
                {"source_forearm_axis", "+X"},
                {"target_forearm_axis", "-Z"},
            }},
            {"right_elbow_joint", {
                {"target", "r_elbow_pitch_joint"},
                {"scale", 1.0},
                {"offset_radians", kG1ToBumi3ElbowZeroOffset},
                {"source_forearm_axis", "+X"},
                {"target_forearm_axis", "-Z"},
            }},
        };
        output["robot_root_name"] = "base_link";
        output["human_root_name"] = "pelvis";
        output["ground_height"] = 0.0;
        output["human_height_assumption"] = 1.0;
        output["use_ik_match_table1"] = true;
        output["use_ik_match_table2"] = true;
        output["human_scale_table"] = nlohmann::json::object();
        for (const auto& [name, scale] : scales)
            output["human_scale_table"][name] = scale;

        for (const std::string table : {"ik_match_table1", "ik_match_table2"}) {
            output[table] = nlohmann::json::object();
            for (const Mapping& mapping : kMappings) {
                const auto& reference_entry = reference.at(table).at(mapping.target);
                output[table][mapping.target] = nlohmann::json::array({
                    mapping.source,
                    reference_entry.at(1).get<double>(),
                    reference_entry.at(2).get<double>(),
                    vectorJson(position_offsets.at(mapping.source)),
                    quaternionJson(rotation_offsets.at(mapping.source)),
                });
            }
        }

        std::ofstream file(output_path);
        if (!file) throw std::runtime_error("cannot create config: " + output_path);
        file << std::setw(2) << output << '\n';
        if (!file) throw std::runtime_error("failed writing config: " + output_path);
        std::cout << "[generate] source sole=" << source_ground
                  << " target sole=" << target_ground
                  << " root_scale=" << root_scale << "\n"
                  << "[generate] contact sole points source/target=4/4 per foot\n"
                  << "[generate] wrote " << output_path
                  << " mappings=" << kMappings.size() << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "generate_g1_to_bumi3_config: " << error.what() << '\n';
        return 1;
    }
}
