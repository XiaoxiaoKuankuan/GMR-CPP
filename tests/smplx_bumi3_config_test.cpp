#include "gmr/body_map.hpp"
#include "gmr/gmr_mink.hpp"

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef BUMI3_REPO_ROOT
#define BUMI3_REPO_ROOT "."
#endif

namespace {

using RobotBodyMap = std::map<std::string, std::string>;

const RobotBodyMap kMapping = {
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
};

std::string readFile(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open " + path);
    std::ostringstream output;
    output << input.rdbuf();
    return output.str();
}

gmr::BodyData pose(double x, double y, double z) {
    gmr::BodyData result;
    result.position = Eigen::Vector3d(x, y, z);
    result.rot_wxyz = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    return result;
}

gmr::BodyMap armsDownPose() {
    return {
        {"pelvis", pose(0.00, 0.00, 0.92)},
        {"spine3", pose(0.00, 0.00, 1.30)},
        {"left_hip", pose(0.00, 0.10, 0.86)},
        {"right_hip", pose(0.00, -0.10, 0.86)},
        {"left_knee", pose(0.00, 0.10, 0.47)},
        {"right_knee", pose(0.00, -0.10, 0.47)},
        {"left_foot", pose(0.10, 0.10, 0.06)},
        {"right_foot", pose(0.10, -0.10, 0.06)},
        {"left_shoulder", pose(0.00, 0.24, 1.34)},
        {"right_shoulder", pose(0.00, -0.24, 1.34)},
        {"left_elbow", pose(0.00, 0.27, 1.08)},
        {"right_elbow", pose(0.00, -0.27, 1.08)},
        {"left_wrist", pose(0.00, 0.28, 0.82)},
        {"right_wrist", pose(0.00, -0.28, 0.82)},
    };
}

std::vector<std::pair<std::string, gmr::BodyMap>> fixedPoses() {
    auto stand = armsDownPose();
    auto t_pose = stand;
    t_pose["left_elbow"].position = Eigen::Vector3d(0.00, 0.52, 1.34);
    t_pose["left_wrist"].position = Eigen::Vector3d(0.00, 0.76, 1.34);
    t_pose["right_elbow"].position = Eigen::Vector3d(0.00, -0.52, 1.34);
    t_pose["right_wrist"].position = Eigen::Vector3d(0.00, -0.76, 1.34);

    auto left_raise = stand;
    left_raise["left_elbow"].position = Eigen::Vector3d(0.00, 0.28, 1.60);
    left_raise["left_wrist"].position = Eigen::Vector3d(0.00, 0.28, 1.84);

    auto right_raise = stand;
    right_raise["right_elbow"].position = Eigen::Vector3d(0.00, -0.28, 1.60);
    right_raise["right_wrist"].position = Eigen::Vector3d(0.00, -0.28, 1.84);

    auto squat = stand;
    squat["pelvis"].position.z() = 0.62;
    squat["spine3"].position.z() = 1.00;
    squat["left_hip"].position.z() = 0.58;
    squat["right_hip"].position.z() = 0.58;
    squat["left_knee"].position = Eigen::Vector3d(0.20, 0.10, 0.35);
    squat["right_knee"].position = Eigen::Vector3d(0.20, -0.10, 0.35);
    squat["left_shoulder"].position.z() = 1.04;
    squat["right_shoulder"].position.z() = 1.04;
    squat["left_elbow"].position.z() = 0.78;
    squat["right_elbow"].position.z() = 0.78;

    auto single_leg = stand;
    single_leg["left_knee"].position = Eigen::Vector3d(0.25, 0.10, 0.72);
    single_leg["left_foot"].position = Eigen::Vector3d(0.35, 0.10, 0.45);

    return {
        {"stand", stand}, {"T-pose", t_pose}, {"arms-down", armsDownPose()},
        {"left-arm-raise", left_raise}, {"right-arm-raise", right_raise},
        {"squat", squat}, {"single-leg-raise", single_leg},
    };
}

double rotationError(const mjtNum* robot_wxyz, const Eigen::Vector4d& target) {
    Eigen::Vector4d robot(
        robot_wxyz[0], robot_wxyz[1], robot_wxyz[2], robot_wxyz[3]);
    const double robot_norm = robot.norm();
    const double target_norm = target.norm();
    if (robot_norm < 1e-12 || target_norm < 1e-12)
        return std::numeric_limits<double>::quiet_NaN();
    double dot = std::abs(robot.dot(target) / (robot_norm * target_norm));
    dot = std::clamp(dot, 0.0, 1.0);
    return 2.0 * std::acos(dot);
}

void validateConfigAndModel(const std::string& root,
                            const std::string& xml_path,
                            const std::string& config_path,
                            const nlohmann::json& config,
                            const mjModel* model) {
    (void)root;
    if (config.at("robot_root_name") != "base_link" ||
        config.at("human_root_name") != "pelvis")
        throw std::runtime_error("unexpected root names");

    const std::set<std::string> supported = {
        "pelvis", "spine3", "left_hip", "right_hip", "left_knee",
        "right_knee", "left_foot", "right_foot", "left_shoulder",
        "right_shoulder", "left_elbow", "right_elbow", "left_wrist",
        "right_wrist",
    };
    std::set<std::string> first_bodies;
    std::set<std::string> second_bodies;
    for (const auto& [table_name, body_set] : std::vector<
             std::pair<std::string, std::set<std::string>*>>{
             {"ik_match_table1", &first_bodies},
             {"ik_match_table2", &second_bodies}}) {
        for (const auto& [robot_body, entry] : config.at(table_name).items()) {
            body_set->insert(robot_body);
            if (mj_name2id(model, mjOBJ_BODY, robot_body.c_str()) < 0)
                throw std::runtime_error("missing robot body: " + robot_body);
            const std::string human = entry.at(0).get<std::string>();
            if (!supported.count(human))
                throw std::runtime_error("unsupported SMP1 target: " + human);
            for (const auto& value : entry.at(3))
                if (!std::isfinite(value.get<double>()))
                    throw std::runtime_error("non-finite position offset");
            double norm_squared = 0.0;
            for (const auto& value : entry.at(4)) {
                const double component = value.get<double>();
                if (!std::isfinite(component))
                    throw std::runtime_error("non-finite quaternion");
                norm_squared += component * component;
            }
            if (std::abs(std::sqrt(norm_squared) - 1.0) > 1e-6)
                throw std::runtime_error("quaternion norm differs from one");
        }
    }
    if (first_bodies != second_bodies || first_bodies.size() != 12)
        throw std::runtime_error("BUMI3 table body sets must match and contain 12 bodies");
    if (first_bodies.count("left_wrist") || first_bodies.count("right_wrist"))
        throw std::runtime_error("wrist must not be used as a robot body");
    for (const auto& [name, scale] : config.at("human_scale_table").items()) {
        (void)name;
        if (!std::isfinite(scale.get<double>()) || scale.get<double>() <= 0.0)
            throw std::runtime_error("human scale must be finite and positive");
    }

    const int base_id = mj_name2id(model, mjOBJ_BODY, "base_link");
    bool base_freejoint = false;
    for (int joint = 0; joint < model->njnt; ++joint) {
        if (model->jnt_bodyid[joint] == base_id &&
            model->jnt_type[joint] == mjJNT_FREE &&
            model->jnt_qposadr[joint] == 0) {
            base_freejoint = true;
        }
    }
    if (base_id < 0 || !base_freejoint || model->nq < 7)
        throw std::runtime_error("base_link root/freejoint validation failed");
    std::cout << "[BUMI3 config] valid fixed runtime config, targets=12, "
                 "XML=" << xml_path << "\n";
}

void validateJumpPreservation(const nlohmann::json& base,
                              const nlohmann::json& jump) {
    for (const char* field : {
             "robot_root_name", "human_root_name", "ground_height",
             "human_height_assumption", "use_ik_match_table1",
             "use_ik_match_table2", "human_scale_table"}) {
        if (base.at(field) != jump.at(field))
            throw std::runtime_error(
                std::string("jump changed protected field: ") + field);
    }
    for (const char* table_name : {"ik_match_table1", "ik_match_table2"}) {
        if (base.at(table_name).size() != jump.at(table_name).size())
            throw std::runtime_error("jump changed robot body count");
        for (const auto& [robot_body, base_entry] :
             base.at(table_name).items()) {
            const auto& jump_entry = jump.at(table_name).at(robot_body);
            if (base_entry.at(0) != jump_entry.at(0) ||
                base_entry.at(3) != jump_entry.at(3) ||
                base_entry.at(4) != jump_entry.at(4)) {
                throw std::runtime_error(
                    std::string("jump changed mapping/offset: ") + table_name +
                    "/" + robot_body);
            }
        }
    }
    std::cout << "[BUMI3 jump] scale/mapping/position-offset/rotation-offset "
                 "preserved\n";
}

double lowestFootTarget(const gmr::BodyMap& targets) {
    return std::min(targets.at("left_foot").position.z(),
                    targets.at("right_foot").position.z());
}

void validateJumpTranslation(const std::string& xml_path,
                             const std::string& jump_config_path) {
    auto frame_a = armsDownPose();
    auto frame_b = frame_a;
    for (auto& [name, body] : frame_b) {
        (void)name;
        body.position.z() += 0.25;
    }

    constexpr double kFixedJumpGroundOffset = 0.65;
    gmr_mink::GMR solver(xml_path, jump_config_path, 1.8, 1.0, false);
    solver.setGroundOffset(kFixedJumpGroundOffset);
    Eigen::VectorXd qpos_a;
    for (int iteration = 0; iteration < 200; ++iteration)
        qpos_a = solver.retarget(frame_a, false);
    const double foot_a = lowestFootTarget(solver.getScaledHumanData());
    constexpr double kCanonicalFootBeforeFixedOffset = 0.5508475377930858;
    const double expected_foot_a =
        kCanonicalFootBeforeFixedOffset - kFixedJumpGroundOffset;
    if (std::abs(foot_a - expected_foot_a) > 1e-6)
        throw std::runtime_error(
            "fixed jump ground offset was not applied uniformly: " +
            std::to_string(foot_a));

    char model_error[1024] = {};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> jump_model(
        mj_loadXML(xml_path.c_str(), nullptr, model_error, sizeof(model_error)),
        mj_deleteModel);
    if (!jump_model) throw std::runtime_error(model_error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> jump_data(
        mj_makeData(jump_model.get()), mj_deleteData);
    for (int index = 0; index < jump_model->nq; ++index)
        jump_data->qpos[index] = qpos_a[index];
    mj_forward(jump_model.get(), jump_data.get());
    const int left_ankle =
        mj_name2id(jump_model.get(), mjOBJ_BODY, "l_ankle_roll_link");
    const int right_ankle =
        mj_name2id(jump_model.get(), mjOBJ_BODY, "r_ankle_roll_link");
    const double robot_foot_a = std::min(
        jump_data->xpos[3 * left_ankle + 2],
        jump_data->xpos[3 * right_ankle + 2]);
    if (!std::isfinite(robot_foot_a))
        throw std::runtime_error(
            "fixed jump ground offset produced invalid robot ankle height: " +
            std::to_string(robot_foot_a));

    Eigen::VectorXd qpos_b;
    for (int iteration = 0; iteration < 200; ++iteration)
        qpos_b = solver.retarget(frame_b, false);
    const double foot_b = lowestFootTarget(solver.getScaledHumanData());

    if (!qpos_a.allFinite() || !qpos_b.allFinite())
        throw std::runtime_error("jump translation produced non-finite qpos");
    const double base_delta = qpos_b[2] - qpos_a[2];
    const double foot_delta = foot_b - foot_a;
    if (!(base_delta > 0.0) || !(foot_delta > 0.0))
        throw std::runtime_error(
            "jump vertical translation was removed: base_delta=" +
            std::to_string(base_delta) + " foot_delta=" +
            std::to_string(foot_delta));
    std::cout << "[BUMI3 jump] vertical translation preserved: base_delta_z="
              << base_delta << " standing_lowest_foot_z=" << foot_a
              << " standing_robot_ankle_z=" << robot_foot_a
              << " lowest_foot_target_delta_z=" << foot_delta << "\n";
}

void reportFixedPose(const std::string& name,
                     const gmr::BodyMap& input,
                     const std::string& xml_path,
                     const std::string& config_path) {
    gmr_mink::GMR solver(xml_path, config_path, 1.8, 1.0, false);
    Eigen::VectorXd qpos;
    for (int iteration = 0; iteration < 100; ++iteration)
        qpos = solver.retarget(input, false);
    if (!qpos.allFinite()) throw std::runtime_error(name + " produced non-finite qpos");

    char error[1024] = {};
    mjModel* raw_model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!raw_model) throw std::runtime_error(error);
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(raw_model, mj_deleteModel);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        mj_makeData(model.get()), mj_deleteData);
    if (qpos.size() != model->nq)
        throw std::runtime_error(name + " qpos size differs from nq");
    for (int i = 0; i < model->nq; ++i) data->qpos[i] = qpos[i];
    mj_forward(model.get(), data.get());

    int limit_hits = 0;
    for (int joint = 0; joint < model->njnt; ++joint) {
        if (!model->jnt_limited[joint] || model->jnt_type[joint] == mjJNT_FREE)
            continue;
        const int address = model->jnt_qposadr[joint];
        const double value = data->qpos[address];
        if (std::abs(value - model->jnt_range[2 * joint]) < 1e-4 ||
            std::abs(value - model->jnt_range[2 * joint + 1]) < 1e-4) {
            ++limit_hits;
        }
    }

    const gmr::BodyMap& targets = solver.getScaledHumanData();
    double position_error = 0.0;
    double rotation_error = 0.0;
    int error_count = 0;
    for (const auto& [human_name, robot_name] : kMapping) {
        const auto target = targets.find(human_name);
        const int body_id = mj_name2id(model.get(), mjOBJ_BODY, robot_name.c_str());
        if (target == targets.end() || body_id < 0) continue;
        const mjtNum* body_position = data->xpos + 3 * body_id;
        position_error +=
            (target->second.position - Eigen::Vector3d(
                body_position[0], body_position[1], body_position[2])).norm();
        rotation_error += rotationError(data->xquat + 4 * body_id,
                                       target->second.rot_wxyz);
        ++error_count;
    }

    const auto height = [&](const char* body_name) {
        const int id = mj_name2id(model.get(), mjOBJ_BODY, body_name);
        return id >= 0 ? data->xpos[3 * id + 2] :
                         std::numeric_limits<double>::quiet_NaN();
    };
    std::cout << "[fixed-pose] " << name << "\n  qpos=" << std::setprecision(6)
              << qpos.transpose() << "\n  joint_limit_hits=" << limit_hits
              << " target_position_error_mean="
              << position_error / std::max(error_count, 1)
              << " target_rotation_error_mean="
              << rotation_error / std::max(error_count, 1)
              << "\n  body_z(base/left_foot/right_foot)="
              << height("base_link") << "/" << height("l_ankle_roll_link")
              << "/" << height("r_ankle_roll_link") << "\n";
}

} // namespace

int main() {
    try {
        const std::string root = BUMI3_REPO_ROOT;
        const std::string xml_path = root + "/assets/bumi3/mjcf/bumi3.xml";
        const std::string config_path =
            root + "/config/ik_configs/smplx_to_bumi3.json";
        const std::string jump_config_path =
            root + "/config/ik_configs/smplx_to_bumi3_jump.json";
        const nlohmann::json config = nlohmann::json::parse(readFile(config_path));
        const nlohmann::json jump_config =
            nlohmann::json::parse(readFile(jump_config_path));

        char error[1024] = {};
        mjModel* raw_model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
        if (!raw_model) throw std::runtime_error(error);
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            raw_model, mj_deleteModel);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        if (!data || !data->qpos)
            throw std::runtime_error("MuJoCo qpos allocation failed");

        validateConfigAndModel(root, xml_path, config_path, config, model.get());
        validateJumpPreservation(config, jump_config);
        validateJumpTranslation(xml_path, jump_config_path);
        for (const auto& [name, frame] : fixedPoses())
            reportFixedPose(name, frame, xml_path, config_path);
        std::cout << "smplx_bumi3_config_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "smplx_bumi3_config_test: FAIL: " << error.what() << "\n";
        return 1;
    }
}
