#include "adapters/g1_motion_adapter.hpp"
#include "gmr/geometry_ground.hpp"
#include "gmr/gmr_mink.hpp"

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef G1_BUMI3_REPO_ROOT
#define G1_BUMI3_REPO_ROOT "."
#endif

namespace {

nlohmann::json readJson(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open " + path);
    nlohmann::json value;
    input >> value;
    return value;
}

struct ModelDeleter { void operator()(mjModel* value) const { mj_deleteModel(value); } };
struct DataDeleter { void operator()(mjData* value) const { mj_deleteData(value); } };

int jointQposAddress(const mjModel* model, const std::string& name) {
    const int joint = mj_name2id(model, mjOBJ_JOINT, name.c_str());
    if (joint < 0) throw std::runtime_error("missing joint: " + name);
    return model->jnt_qposadr[joint];
}

int sourceJointIndex(const std::string& name) {
    const auto& names = gmr::G1MotionAdapter::omg_joint_order();
    const auto found = std::find(names.begin(), names.end(), name);
    if (found == names.end()) throw std::runtime_error("missing source joint: " + name);
    return static_cast<int>(std::distance(names.begin(), found));
}

Eigen::Vector3d bodyAxis(const mjModel* model, const mjData* data,
                         const std::string& body_name,
                         const Eigen::Vector3d& local_axis) {
    const int body = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body < 0) throw std::runtime_error("missing body: " + body_name);
    Eigen::Matrix3d rotation;
    const mjtNum* matrix = data->xmat + 9 * body;
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            rotation(row, column) = matrix[3 * row + column];
    return (rotation * local_axis).normalized();
}

}  // namespace

int main() {
    try {
        const std::string root = G1_BUMI3_REPO_ROOT;
        const std::string source_xml =
            root + "/assets/unitree_g1/g1_mocap_29dof.xml";
        const std::string target_xml = root + "/assets/bumi3/mjcf/bumi3.xml";
        const std::string config_path =
            root + "/config/ik_configs/g1_to_bumi3.json";
        const std::string reference_path =
            root + "/config/ik_configs/smplx_to_bumi3.json";
        const nlohmann::json config = readJson(config_path);
        const nlohmann::json reference = readJson(reference_path);
        if (config.at("source").at("robot") != "unitree_g1" ||
            config.at("target").at("robot") != "bumi3")
            throw std::runtime_error("source/target metadata mismatch");
        if (config.at("root").at("source") != "pelvis" ||
            config.at("root").at("target") != "base_link" ||
            !config.at("root").at("ground_alignment").get<bool>() ||
            config.at("root").at("height_offset").get<double>() != 0.0)
            throw std::runtime_error("root policy mismatch");
        if (config.at("body_mapping").size() != 12)
            throw std::runtime_error("body mapping must contain 12 pairs");
        const auto& semantic = config.at("semantic_joint_mapping");
        for (const char* side : {"left_elbow_joint", "right_elbow_joint"}) {
            const auto& entry = semantic.at(side);
            if (entry.at("scale").get<double>() != 1.0 ||
                std::abs(entry.at("offset_radians").get<double>() +
                         0.5 * std::acos(-1.0)) > 1e-12 ||
                entry.at("source_forearm_axis") != "+X" ||
                entry.at("target_forearm_axis") != "-Z")
                throw std::runtime_error("semantic elbow mapping metadata mismatch");
        }

        char error[1024] = {};
        std::unique_ptr<mjModel, ModelDeleter> source_model(
            mj_loadXML(source_xml.c_str(), nullptr, error, sizeof(error)));
        if (!source_model) throw std::runtime_error(error);
        std::unique_ptr<mjModel, ModelDeleter> target_model(
            mj_loadXML(target_xml.c_str(), nullptr, error, sizeof(error)));
        if (!target_model) throw std::runtime_error(error);
        std::unique_ptr<mjData, DataDeleter> target_data(
            mj_makeData(target_model.get()));
        if (!target_data) throw std::runtime_error("cannot allocate target data");
        for (const auto& [source, target] : config.at("body_mapping").items()) {
            const std::string target_name = target.get<std::string>();
            if (mj_name2id(source_model.get(), mjOBJ_BODY, source.c_str()) < 0 ||
                mj_name2id(target_model.get(), mjOBJ_BODY,
                           target_name.c_str()) < 0)
                throw std::runtime_error("mapping references missing body: " + source);
            for (const char* table : {"ik_match_table1", "ik_match_table2"}) {
                const auto& entry = config.at(table).at(target_name);
                const auto& weight = reference.at(table).at(target_name);
                if (entry.at(0) != source || entry.at(1) != weight.at(1) ||
                    entry.at(2) != weight.at(2))
                    throw std::runtime_error("generated weights/mapping mismatch: " + source);
            }
        }

        gmr::G1MotionFrame frame;
        frame.root_position = Eigen::Vector3d(0.0, 0.0, 0.793);
        frame.root_rotation = Eigen::Quaterniond::Identity();
        frame.joint_positions.assign(29, 0.0);
        gmr::G1MotionAdapter adapter(source_xml);
        const gmr::BodyMap bodies = adapter.to_body_map(frame);
        gmr_mink::GMR solver(target_xml, config_path, 1.0, 1.0, false);
        Eigen::VectorXd qpos;
        for (int iteration = 0; iteration < 100; ++iteration)
            qpos = solver.retarget(bodies, false);
        gmr::TargetGroundAligner aligner(
            target_xml,
            config.at("root").at("target_foot_bodies")
                .get<std::vector<std::string>>());
        aligner.align(qpos, 0.0);
        if (qpos.size() != 28 || !qpos.allFinite())
            throw std::runtime_error("neutral retarget output invalid");
        const double sole = aligner.lowest(qpos);
        if (std::abs(sole) > 1e-9)
            throw std::runtime_error("model-derived ground alignment failed: " +
                                     std::to_string(sole));
        Eigen::VectorXd visually_seated = qpos;
        aligner.align(visually_seated, -0.005);
        const double seated_sole = aligner.lowest(visually_seated);
        if (std::abs(seated_sole + 0.005) > 1e-9)
            throw std::runtime_error(
                "visual ground penetration failed: " +
                std::to_string(seated_sole));

        // This is the regression that the rest-frame-only configuration
        // missed.  G1 elbow-local +X and BUMI3 elbow-local -Z are the physical
        // forearm directions, so matching motion requires q_bumi=q_g1-pi/2.
        const int left_source_elbow = sourceJointIndex("left_elbow_joint");
        const int right_source_elbow = sourceJointIndex("right_elbow_joint");
        const int left_target_elbow =
            jointQposAddress(target_model.get(), "l_elbow_pitch_joint");
        const int right_target_elbow =
            jointQposAddress(target_model.get(), "r_elbow_pitch_joint");
        const double half_pi = 0.5 * std::acos(-1.0);
        double worst_angle_error = 0.0;
        for (const double source_elbow : {-0.4, 0.0, 0.5, 1.0, 1.5}) {
            frame.joint_positions.assign(29, 0.0);
            frame.joint_positions[left_source_elbow] = source_elbow;
            frame.joint_positions[right_source_elbow] = source_elbow;
            const gmr::BodyMap pose_bodies = adapter.to_body_map(frame);
            gmr_mink::GMR pose_solver(
                target_xml, config_path, 1.0, 1.0, false);
            Eigen::VectorXd pose_qpos;
            for (int iteration = 0; iteration < 100; ++iteration)
                pose_qpos = pose_solver.retarget(pose_bodies, false);
            const double expected = source_elbow - half_pi;
            for (const int address : {left_target_elbow, right_target_elbow}) {
                const double error_radians =
                    std::abs(pose_qpos[address] - expected);
                if (error_radians > 0.06)
                    throw std::runtime_error(
                        "semantic elbow angle error too large: " +
                        std::to_string(error_radians));
            }

            for (int index = 0; index < target_model->nq; ++index)
                target_data->qpos[index] = pose_qpos[index];
            mj_forward(target_model.get(), target_data.get());
            for (const auto& [source_body, target_body] : {
                    std::pair<std::string, std::string>{
                        "left_elbow_link", "l_elbow_pitch_link"},
                    {"right_elbow_link", "r_elbow_pitch_link"}}) {
                const Eigen::Vector3d source_forearm = bodyAxis(
                    adapter.model(), adapter.data(), source_body,
                    Eigen::Vector3d::UnitX());
                const Eigen::Vector3d target_forearm = bodyAxis(
                    target_model.get(), target_data.get(), target_body,
                    -Eigen::Vector3d::UnitZ());
                const double angle_error = std::acos(std::clamp(
                    source_forearm.dot(target_forearm), -1.0, 1.0));
                worst_angle_error = std::max(worst_angle_error, angle_error);
                if (angle_error > 0.10)
                    throw std::runtime_error(
                        "semantic forearm direction error too large: " +
                        std::to_string(angle_error));
            }
        }
        std::cout << "g1_to_bumi3_config_test: PASS mappings=12 sole_z="
                  << sole << " seated_sole_z=" << seated_sole
                  << " worst_forearm_error_deg="
                  << worst_angle_error * 180.0 / std::acos(-1.0) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "g1_to_bumi3_config_test: FAIL: " << error.what() << '\n';
        return 1;
    }
}
