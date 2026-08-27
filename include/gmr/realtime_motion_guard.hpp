#pragma once
/**
 * 实时关节轨迹安全保护器。
 *
 * 本文件用于实时 SMPL-X 重定向链路：IK 仍然按原流程计算当前目标姿势，随后只对
 * MuJoCo 中的单自由度关节做一次 O(N) 的因果检查。保护器不读取未来帧、不睡眠、
 * 不重复运行 IK，因此不会引入与动作长度相关的计算量。它提供三层保护：
 *
 * 1. 关节速度硬限制：相邻输出之间的变化量不能超过“最大速度 × 实际帧间隔”；
 * 2. 关节加速度硬限制：本帧速度相对上一帧速度的变化不能超过设定加速度；
 * 3. 左右手臂换解保护：如果整条手臂突然跳到相差很大的 IK 分支，先保持一帧并
 *    等待下一帧确认。单帧识别毛刺会被丢弃，连续真实动作则在确认后按速度、
 *    加速度上限平滑追赶。
 *
 * freejoint 的根位置和四元数不在这里修改，落地、支撑脚和腾空由已有脚接触约束
 * 负责。类中只在初始化时解析模型和分配内存，apply() 的实时路径不会增减容器。
 */

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <vector>

namespace gmr {

struct RealtimeMotionGuardConfig {
    double nominal_timestep = 0.02;
    double minimum_timestep = 0.005;
    double maximum_timestep = 0.05;
    double max_joint_velocity = 12.0;
    double max_joint_acceleration = 80.0;
    double max_arm_velocity = 12.0;
    double max_arm_acceleration = 60.0;
    double arm_jump_threshold = 0.60;
    double arm_jump_release_threshold = 0.25;
    double arm_jump_candidate_tolerance = 0.20;
    int arm_jump_confirmation_frames = 2;
    bool protect_non_arm_joints = true;
    std::vector<std::string> left_arm_joints;
    std::vector<std::string> right_arm_joints;
    std::unordered_map<std::string, double> joint_velocity_limits;
};

struct RealtimeMotionGuardDiagnostics {
    double timestep = 0.0;
    double maximum_raw_arm_delta = 0.0;
    int velocity_limited_joints = 0;
    int acceleration_limited_joints = 0;
    bool left_arm_jump_held = false;
    bool right_arm_jump_held = false;
    bool left_arm_jump_confirmed = false;
    bool right_arm_jump_confirmed = false;
    uint64_t total_velocity_limited = 0;
    uint64_t total_acceleration_limited = 0;
    uint64_t total_arm_jump_holds = 0;
    uint64_t total_arm_jump_confirmations = 0;
};

class RealtimeMotionGuard {
public:
    RealtimeMotionGuard(const mjModel* model, RealtimeMotionGuardConfig config)
        : config_(std::move(config)) {
        validateConfig();
        if (!model) throw std::runtime_error("realtime motion guard model is null");
        nq_ = model->nq;

        const std::unordered_set<std::string> left_names(
            config_.left_arm_joints.begin(), config_.left_arm_joints.end());
        const std::unordered_set<std::string> right_names(
            config_.right_arm_joints.begin(), config_.right_arm_joints.end());
        for (int joint = 0; joint < model->njnt; ++joint) {
            const int type = model->jnt_type[joint];
            if (type != mjJNT_HINGE && type != mjJNT_SLIDE) continue;
            const char* raw_name = mj_id2name(model, mjOBJ_JOINT, joint);
            const std::string name = raw_name ? raw_name : "";
            Joint record;
            record.qpos_address = model->jnt_qposadr[joint];
            record.arm_side = left_names.count(name) ? 0 :
                              right_names.count(name) ? 1 : -1;
            record.velocity_limit = record.arm_side >= 0
                ? config_.max_arm_velocity : config_.max_joint_velocity;
            const auto configured_limit =
                config_.joint_velocity_limits.find(name);
            if (configured_limit != config_.joint_velocity_limits.end())
                record.velocity_limit = configured_limit->second;
            if (model->jnt_limited[joint]) {
                record.lower = model->jnt_range[2 * joint];
                record.upper = model->jnt_range[2 * joint + 1];
            }
            const size_t index = joints_.size();
            joints_.push_back(record);
            if (record.arm_side == 0) left_arm_indices_.push_back(index);
            if (record.arm_side == 1) right_arm_indices_.push_back(index);
        }
        validateArmNames(model, left_names, left_arm_indices_, "left");
        validateArmNames(model, right_names, right_arm_indices_, "right");
        validateVelocityLimitNames(model);

        previous_qpos_.resize(nq_);
        previous_velocity_.assign(joints_.size(), 0.0);
        left_arm_.pending_values.resize(left_arm_indices_.size(), 0.0);
        right_arm_.pending_values.resize(right_arm_indices_.size(), 0.0);
    }

    void reset(const Eigen::Ref<const Eigen::VectorXd>& qpos) {
        if (qpos.size() != nq_ || !qpos.allFinite())
            throw std::runtime_error("realtime motion guard reset qpos is invalid");
        previous_qpos_ = qpos;
        std::fill(previous_velocity_.begin(), previous_velocity_.end(), 0.0);
        left_arm_ = resetArmState(left_arm_indices_.size());
        right_arm_ = resetArmState(right_arm_indices_.size());
        previous_timestamp_ = std::numeric_limits<double>::quiet_NaN();
        initialized_ = true;
        diagnostics_ = {};
    }

    void apply(Eigen::Ref<Eigen::VectorXd> qpos, double timestamp) {
        if (qpos.size() != nq_ || !qpos.allFinite())
            throw std::runtime_error("realtime motion guard qpos is invalid");
        if (!initialized_) {
            reset(qpos);
            return;
        }

        diagnostics_.timestep = chooseTimestep(timestamp);
        diagnostics_.maximum_raw_arm_delta = 0.0;
        diagnostics_.velocity_limited_joints = 0;
        diagnostics_.acceleration_limited_joints = 0;
        diagnostics_.left_arm_jump_held = false;
        diagnostics_.right_arm_jump_held = false;
        diagnostics_.left_arm_jump_confirmed = false;
        diagnostics_.right_arm_jump_confirmed = false;

        protectArm(qpos, left_arm_indices_, left_arm_, true,
                   diagnostics_.timestep);
        protectArm(qpos, right_arm_indices_, right_arm_, false,
                   diagnostics_.timestep);

        const double dt = diagnostics_.timestep;
        for (size_t index = 0; index < joints_.size(); ++index) {
            const Joint& joint = joints_[index];
            const double previous = previous_qpos_[joint.qpos_address];
            const double desired_velocity =
                (qpos[joint.qpos_address] - previous) / dt;
            const bool arm = joint.arm_side >= 0;
            // Batch already constrains legs and waist together with root/contact
            // inside the shared QP.  In that mode this post-QP guard is kept only
            // for arm branch confirmation; touching a leg here would invalidate
            // the sole-plane feasibility that the QP just established.
            if (!config_.protect_non_arm_joints && !arm) {
                previous_velocity_[index] = desired_velocity;
                continue;
            }
            const double velocity_limit = joint.velocity_limit;
            const double acceleration_limit = arm ? config_.max_arm_acceleration :
                                                    config_.max_joint_acceleration;
            const double velocity_clamped = std::clamp(
                desired_velocity, -velocity_limit, velocity_limit);
            if (std::abs(velocity_clamped - desired_velocity) > 1e-12) {
                ++diagnostics_.velocity_limited_joints;
                ++diagnostics_.total_velocity_limited;
            }

            const double acceleration_step = acceleration_limit * dt;
            const double lower_velocity = std::max(
                -velocity_limit, previous_velocity_[index] - acceleration_step);
            const double upper_velocity = std::min(
                velocity_limit, previous_velocity_[index] + acceleration_step);
            const double output_velocity = std::clamp(
                velocity_clamped, lower_velocity, upper_velocity);
            if (std::abs(output_velocity - velocity_clamped) > 1e-12) {
                ++diagnostics_.acceleration_limited_joints;
                ++diagnostics_.total_acceleration_limited;
            }

            const double output = std::clamp(
                previous + output_velocity * dt, joint.lower, joint.upper);
            qpos[joint.qpos_address] = output;
            previous_velocity_[index] = (output - previous) / dt;
        }
        previous_qpos_ = qpos;
    }

    const RealtimeMotionGuardDiagnostics& diagnostics() const {
        return diagnostics_;
    }

private:
    struct Joint {
        int qpos_address = -1;
        int arm_side = -1;
        double lower = -std::numeric_limits<double>::infinity();
        double upper = std::numeric_limits<double>::infinity();
        double velocity_limit = 0.0;
    };

    struct ArmState {
        int pending_frames = 0;
        bool recovery_active = false;
        std::vector<double> pending_values;
    };

    static ArmState resetArmState(size_t size) {
        ArmState result;
        result.pending_values.resize(size, 0.0);
        return result;
    }

    void validateConfig() const {
        const bool valid_timestep =
            std::isfinite(config_.nominal_timestep) &&
            std::isfinite(config_.minimum_timestep) &&
            std::isfinite(config_.maximum_timestep) &&
            config_.minimum_timestep > 0.0 &&
            config_.nominal_timestep >= config_.minimum_timestep &&
            config_.nominal_timestep <= config_.maximum_timestep &&
            config_.maximum_timestep <= 0.2;
        const bool valid_limits =
            std::isfinite(config_.max_joint_velocity) &&
            config_.max_joint_velocity > 0.0 &&
            std::isfinite(config_.max_joint_acceleration) &&
            config_.max_joint_acceleration > 0.0 &&
            std::isfinite(config_.max_arm_velocity) &&
            config_.max_arm_velocity > 0.0 &&
            std::isfinite(config_.max_arm_acceleration) &&
            config_.max_arm_acceleration > 0.0;
        const bool valid_joint_limits = std::all_of(
            config_.joint_velocity_limits.begin(),
            config_.joint_velocity_limits.end(),
            [](const auto& entry) {
                return !entry.first.empty() && std::isfinite(entry.second) &&
                       entry.second > 0.0;
            });
        const bool valid_jump =
            std::isfinite(config_.arm_jump_threshold) &&
            config_.arm_jump_threshold > 0.0 &&
            std::isfinite(config_.arm_jump_release_threshold) &&
            config_.arm_jump_release_threshold >= 0.0 &&
            config_.arm_jump_release_threshold < config_.arm_jump_threshold &&
            std::isfinite(config_.arm_jump_candidate_tolerance) &&
            config_.arm_jump_candidate_tolerance >= 0.0 &&
            config_.arm_jump_confirmation_frames >= 2;
        if (!valid_timestep || !valid_limits || !valid_joint_limits ||
            !valid_jump)
            throw std::runtime_error("invalid realtime motion guard configuration");
    }

    void validateVelocityLimitNames(const mjModel* model) const {
        for (const auto& [name, limit] : config_.joint_velocity_limits) {
            (void)limit;
            const int joint = mj_name2id(model, mjOBJ_JOINT, name.c_str());
            if (joint < 0 ||
                (model->jnt_type[joint] != mjJNT_HINGE &&
                 model->jnt_type[joint] != mjJNT_SLIDE))
                throw std::runtime_error(
                    "realtime velocity-limit joint is missing or not 1-DoF: " +
                    name);
        }
    }

    static void validateArmNames(
        const mjModel* model, const std::unordered_set<std::string>& requested,
        const std::vector<size_t>& resolved, const char* side) {
        if (requested.empty())
            throw std::runtime_error(std::string(side) +
                                     " arm joint list must not be empty");
        if (resolved.size() != requested.size()) {
            std::vector<std::string> missing;
            for (const auto& name : requested)
                if (mj_name2id(model, mjOBJ_JOINT, name.c_str()) < 0)
                    missing.push_back(name);
            std::string message = std::string(side) + " arm joints unresolved";
            for (const auto& name : missing) message += " " + name;
            throw std::runtime_error(message);
        }
    }

    double chooseTimestep(double timestamp) {
        double result = config_.nominal_timestep;
        if (std::isfinite(timestamp) && std::isfinite(previous_timestamp_) &&
            timestamp > previous_timestamp_) {
            result = std::clamp(timestamp - previous_timestamp_,
                                config_.minimum_timestep,
                                config_.maximum_timestep);
        }
        if (std::isfinite(timestamp)) previous_timestamp_ = timestamp;
        return result;
    }

    void protectArm(Eigen::Ref<Eigen::VectorXd> qpos,
                    const std::vector<size_t>& indices,
                    ArmState& state, bool left, double dt) {
        double maximum_delta = 0.0;
        for (const size_t joint_index : indices) {
            const int address = joints_[joint_index].qpos_address;
            maximum_delta = std::max(
                maximum_delta,
                std::abs(qpos[address] - previous_qpos_[address]));
        }
        diagnostics_.maximum_raw_arm_delta = std::max(
            diagnostics_.maximum_raw_arm_delta, maximum_delta);

        if (state.recovery_active) {
            if (maximum_delta <= config_.arm_jump_release_threshold)
                state.recovery_active = false;
            return;
        }
        if (maximum_delta <= config_.arm_jump_threshold) {
            state.pending_frames = 0;
            return;
        }

        bool same_candidate = state.pending_frames > 0;
        if (same_candidate) {
            double candidate_change = 0.0;
            for (size_t slot = 0; slot < indices.size(); ++slot) {
                const int address = joints_[indices[slot]].qpos_address;
                candidate_change = std::max(
                    candidate_change,
                    std::abs(qpos[address] - state.pending_values[slot]));
            }
            same_candidate =
                candidate_change <= config_.arm_jump_candidate_tolerance;
        }
        if (same_candidate) {
            ++state.pending_frames;
        } else {
            state.pending_frames = 1;
            for (size_t slot = 0; slot < indices.size(); ++slot)
                state.pending_values[slot] =
                    qpos[joints_[indices[slot]].qpos_address];
        }

        if (state.pending_frames < config_.arm_jump_confirmation_frames) {
            for (const size_t joint_index : indices) {
                const int address = joints_[joint_index].qpos_address;
                qpos[address] = previous_qpos_[address] +
                                previous_velocity_[joint_index] * dt;
            }
            if (left) diagnostics_.left_arm_jump_held = true;
            else diagnostics_.right_arm_jump_held = true;
            ++diagnostics_.total_arm_jump_holds;
            return;
        }

        state.pending_frames = 0;
        state.recovery_active = true;
        if (left) diagnostics_.left_arm_jump_confirmed = true;
        else diagnostics_.right_arm_jump_confirmed = true;
        ++diagnostics_.total_arm_jump_confirmations;
    }

    RealtimeMotionGuardConfig config_;
    int nq_ = 0;
    std::vector<Joint> joints_;
    std::vector<size_t> left_arm_indices_;
    std::vector<size_t> right_arm_indices_;
    Eigen::VectorXd previous_qpos_;
    std::vector<double> previous_velocity_;
    ArmState left_arm_;
    ArmState right_arm_;
    bool initialized_ = false;
    double previous_timestamp_ = std::numeric_limits<double>::quiet_NaN();
    RealtimeMotionGuardDiagnostics diagnostics_;
};

}  // namespace gmr
