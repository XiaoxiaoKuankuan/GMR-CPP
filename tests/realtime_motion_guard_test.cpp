/**
 * 实时关节保护器的确定性单元测试。
 *
 * 测试直接加载仓库中的 BUMI3 MuJoCo 模型，使用真实关节名称和 qpos 地址验证：
 * 速度上限、加速度上限、左右手臂首帧跳变保持、第二帧连续目标确认，以及保护后
 * 结果仍保持有限数值。测试不启动 UDP、Redis 或 Viewer，失败时返回非零状态，适合
 * 在每次编译后由 CTest 自动执行。
 */

#include "gmr/realtime_motion_guard.hpp"

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

int qposAddress(const mjModel* model, const std::string& name) {
    const int joint = mj_name2id(model, mjOBJ_JOINT, name.c_str());
    if (joint < 0) throw std::runtime_error("missing test joint: " + name);
    return model->jnt_qposadr[joint];
}

gmr::RealtimeMotionGuardConfig config() {
    gmr::RealtimeMotionGuardConfig result;
    result.left_arm_joints = {
        "l_arm_pitch_joint", "l_arm_roll_joint",
        "l_arm_yaw_joint", "l_elbow_pitch_joint"};
    result.right_arm_joints = {
        "r_arm_pitch_joint", "r_arm_roll_joint",
        "r_arm_yaw_joint", "r_elbow_pitch_joint"};
    result.joint_velocity_limits = {{"waist_yaw_joint", 9.0}};
    return result;
}

}  // namespace

int main() {
    try {
        const std::string xml =
            std::string(BUMI3_REPO_ROOT) + "/assets/bumi3/mjcf/bumi3.xml";
        char error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(xml.c_str(), nullptr, error, sizeof(error)),
            mj_deleteModel);
        if (!model) throw std::runtime_error(error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
            mj_makeData(model.get()), mj_deleteData);
        if (!data) throw std::runtime_error("mj_makeData failed");

        Eigen::VectorXd initial(model->nq);
        for (int index = 0; index < model->nq; ++index)
            initial[index] = data->qpos[index];
        require(initial.allFinite(), "initial BUMI3 qpos is not finite");

        gmr::RealtimeMotionGuard guard(model.get(), config());
        guard.reset(initial);
        const int waist = qposAddress(model.get(), "waist_yaw_joint");
        Eigen::VectorXd first = initial;
        first[waist] += 0.5;
        guard.apply(first, 1.02);
        const double first_velocity = (first[waist] - initial[waist]) / 0.02;
        require(std::abs(first_velocity) <= 9.0 + 1e-9,
                "joint velocity limit was exceeded on first frame");
        require(std::abs(first_velocity) <= 80.0 * 0.02 + 1e-9,
                "joint acceleration limit was exceeded from rest");

        Eigen::VectorXd second = initial;
        second[waist] += 0.5;
        guard.apply(second, 1.04);
        const double second_velocity = (second[waist] - first[waist]) / 0.02;
        require(std::abs(second_velocity) <= 9.0 + 1e-9,
                "joint velocity limit was exceeded on second frame");
        require(std::abs(second_velocity - first_velocity) <=
                    80.0 * 0.02 + 1e-9,
                "joint acceleration limit was exceeded between frames");
        require(guard.diagnostics().total_velocity_limited > 0 &&
                    guard.diagnostics().total_acceleration_limited > 0,
                "limit diagnostics did not record intervention");

        auto reference_limit_config = config();
        reference_limit_config.max_joint_acceleration = 10000.0;
        reference_limit_config.max_arm_acceleration = 10000.0;
        gmr::RealtimeMotionGuard reference_limit_guard(
            model.get(), reference_limit_config);
        reference_limit_guard.reset(initial);
        const int left_pitch =
            qposAddress(model.get(), "l_arm_pitch_joint");
        Eigen::VectorXd reference_limit_target = initial;
        reference_limit_target[waist] += 0.5;
        reference_limit_target[left_pitch] += 0.5;
        reference_limit_guard.apply(reference_limit_target, 1.02);
        require(std::abs(
                    (reference_limit_target[waist] - initial[waist]) / 0.02 -
                    9.0) < 1e-9,
                "waist did not use the bumi.py 9 rad/s limit");
        require(std::abs(
                    (reference_limit_target[left_pitch] - initial[left_pitch]) /
                        0.02 - 12.0) < 1e-9,
                "arm did not use the bumi.py 12 rad/s limit");

        guard.reset(initial);
        const std::vector<std::string> left_arm = {
            "l_arm_pitch_joint", "l_arm_roll_joint",
            "l_arm_yaw_joint", "l_elbow_pitch_joint"};
        Eigen::VectorXd jump = initial;
        for (const auto& name : left_arm)
            jump[qposAddress(model.get(), name)] += 0.8;
        guard.apply(jump, 2.02);
        require(guard.diagnostics().left_arm_jump_held,
                "first left-arm branch jump was not held");
        require(!guard.diagnostics().right_arm_jump_held,
                "left-arm jump incorrectly held the right arm");
        for (const auto& name : left_arm) {
            const int address = qposAddress(model.get(), name);
            require(std::abs(jump[address] - initial[address]) < 1e-12,
                    "held arm changed during the confirmation frame");
        }

        Eigen::VectorXd confirmed = initial;
        for (const auto& name : left_arm)
            confirmed[qposAddress(model.get(), name)] += 0.8;
        guard.apply(confirmed, 2.04);
        require(guard.diagnostics().left_arm_jump_confirmed,
                "continuous left-arm target was not confirmed on second frame");
        for (const auto& name : left_arm) {
            const int address = qposAddress(model.get(), name);
            const double velocity =
                (confirmed[address] - initial[address]) / 0.02;
            require(std::abs(velocity) <= 12.0 + 1e-9,
                    "confirmed arm exceeded its velocity limit");
            require(std::abs(velocity) <= 60.0 * 0.02 + 1e-9,
                    "confirmed arm exceeded its acceleration limit");
        }
        require(confirmed.allFinite(), "guard returned non-finite qpos");

        std::cout << "realtime_motion_guard_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "realtime_motion_guard_test: FAIL: "
                  << error.what() << '\n';
        return 1;
    }
}
