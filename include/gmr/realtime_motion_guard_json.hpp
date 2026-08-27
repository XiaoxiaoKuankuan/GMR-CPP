#pragma once
/**
 * 实时轨迹保护配置的统一 JSON 解析入口。
 *
 * 实时 UDP 服务和同步 batch 服务都读取同一份 SMPL-X→BUMI3 IK 配置。此前只有
 * UDP 服务在自己的 cpp 文件里解析 `realtime_safety`，batch 即使看见
 * `enabled=true` 也不会真正构造保护器，因而同一动作在两条链路中具有不同的速度、
 * 加速度和手臂换解语义。本文件把解析逻辑放进共享头文件，使两个入口使用完全相同的
 * 时间步范围、逐关节速度表和左右手臂确认参数。
 *
 * 本文件只负责字段映射；数值合法性、关节是否存在以及左右手臂名称完整性仍由
 * RealtimeMotionGuard 构造函数统一校验，避免两个入口各自维护一套验证规则。
 */

#include "gmr/realtime_motion_guard.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace gmr {

inline RealtimeMotionGuardConfig realtimeGuardConfigFromJson(
    const nlohmann::json& root) {
    if (!root.contains("realtime_safety"))
        throw std::runtime_error("IK config has no realtime_safety profile");
    const auto& value = root.at("realtime_safety");
    RealtimeMotionGuardConfig result;
    result.nominal_timestep = value.value(
        "nominal_timestep_s", result.nominal_timestep);
    result.minimum_timestep = value.value(
        "minimum_timestep_s", result.minimum_timestep);
    result.maximum_timestep = value.value(
        "maximum_timestep_s", result.maximum_timestep);
    result.max_joint_velocity = value.value(
        "max_joint_velocity_rps", result.max_joint_velocity);
    result.max_joint_acceleration = value.value(
        "max_joint_acceleration_rps2", result.max_joint_acceleration);
    result.max_arm_velocity = value.value(
        "max_arm_velocity_rps", result.max_arm_velocity);
    result.max_arm_acceleration = value.value(
        "max_arm_acceleration_rps2", result.max_arm_acceleration);
    result.arm_jump_threshold = value.value(
        "arm_jump_threshold_rad", result.arm_jump_threshold);
    result.arm_jump_release_threshold = value.value(
        "arm_jump_release_threshold_rad",
        result.arm_jump_release_threshold);
    result.arm_jump_candidate_tolerance = value.value(
        "arm_jump_candidate_tolerance_rad",
        result.arm_jump_candidate_tolerance);
    result.arm_jump_confirmation_frames = value.value(
        "arm_jump_confirmation_frames",
        result.arm_jump_confirmation_frames);
    result.left_arm_joints = value.at("left_arm_joints")
        .get<std::vector<std::string>>();
    result.right_arm_joints = value.at("right_arm_joints")
        .get<std::vector<std::string>>();
    if (value.contains("joint_velocity_limits_rps")) {
        for (const auto& [name, limit] :
             value.at("joint_velocity_limits_rps").items())
            result.joint_velocity_limits[name] = limit.get<double>();
    }
    return result;
}

}  // namespace gmr
