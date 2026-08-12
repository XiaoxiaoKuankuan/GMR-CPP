#pragma once

#include "gmr/foot_contact.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace gmr {

inline Eigen::Vector3d footVectorFromJson(const nlohmann::json& value,
                                          const std::string& name) {
    if (!value.is_array() || value.size() != 3)
        throw std::runtime_error(name + " must be a three-element array");
    Eigen::Vector3d result(value.at(0).get<double>(), value.at(1).get<double>(),
                           value.at(2).get<double>());
    if (!result.allFinite()) throw std::runtime_error(name + " contains non-finite values");
    return result;
}

inline FootSoleDefinition footSoleDefinitionFromJson(
    const nlohmann::json& value, const std::string& name) {
    FootSoleDefinition result;
    result.body_name = value.at("body").get<std::string>();
    if (result.body_name.empty()) throw std::runtime_error(name + ".body is empty");
    const auto& points = value.at("sole_points");
    if (!points.is_array() || points.size() != 4)
        throw std::runtime_error(name + ".sole_points must contain four points");
    for (size_t index = 0; index < result.points_local.size(); ++index)
        result.points_local[index] = footVectorFromJson(
            points.at(index), name + ".sole_points[" + std::to_string(index) + "]");
    if ((result.points_local[2] - result.points_local[0]).norm() < 1e-4 ||
        (result.points_local[1] - result.points_local[0]).norm() < 1e-4)
        throw std::runtime_error(name + " sole points are degenerate");
    return result;
}

inline nlohmann::json footSoleDefinitionToJson(
    const FootSoleDefinition& definition) {
    nlohmann::json points = nlohmann::json::array();
    for (const auto& point : definition.points_local)
        points.push_back({point.x(), point.y(), point.z()});
    return {{"body", definition.body_name}, {"sole_points", points}};
}

inline FootContactDetectorConfig footDetectorConfigFromJson(
    const nlohmann::json& foot_contact) {
    FootContactDetectorConfig result;
    result.ground_z = foot_contact.value("ground_z", 0.0);
    result.allow_flight = foot_contact.value("allow_flight", false);
    if (foot_contact.contains("detection")) {
        const auto& value = foot_contact.at("detection");
        result.enter_height = value.value("enter_height_m", result.enter_height);
        result.exit_height = value.value("exit_height_m", result.exit_height);
        result.max_enter_vertical_speed = value.value(
            "max_enter_vertical_speed_mps", result.max_enter_vertical_speed);
        result.max_exit_upward_speed = value.value(
            "max_exit_upward_speed_mps", result.max_exit_upward_speed);
        result.max_enter_horizontal_speed = value.value(
            "max_enter_horizontal_speed_mps",
            result.max_enter_horizontal_speed);
        result.max_exit_horizontal_speed = value.value(
            "max_exit_horizontal_speed_mps",
            result.max_exit_horizontal_speed);
        result.double_support_max_horizontal_speed = value.value(
            "double_support_max_horizontal_speed_mps",
            result.double_support_max_horizontal_speed);
        result.enter_frames = value.value("enter_frames", result.enter_frames);
        result.exit_frames = value.value("exit_frames", result.exit_frames);
        result.minimum_stance_frames = value.value(
            "minimum_stance_frames", result.minimum_stance_frames);
    }
    // Reuse the detector constructor for complete validation.
    FootContactDetector validated(result);
    return validated.config();
}

inline FootConstraintConfig footConstraintConfigFromJson(
    const nlohmann::json& root) {
    FootConstraintConfig result;
    if (!root.contains("foot_contact")) return result;
    const auto& value = root.at("foot_contact");
    result.enabled = value.value("enabled", false);
    result.ground_z = value.value("ground_z", 0.0);
    if (!std::isfinite(result.ground_z))
        throw std::runtime_error("foot_contact.ground_z must be finite");
    if (!value.contains("target")) {
        if (result.enabled)
            throw std::runtime_error("enabled foot_contact config requires target feet");
        return result;
    }
    result.left = footSoleDefinitionFromJson(
        value.at("target").at("left"), "foot_contact.target.left");
    result.right = footSoleDefinitionFromJson(
        value.at("target").at("right"), "foot_contact.target.right");
    if (value.contains("constraints")) {
        const auto& settings = value.at("constraints");
        auto& target = result.settings;
        target.xy_weight = settings.value("xy_weight", target.xy_weight);
        target.tilt_weight = settings.value("tilt_weight", target.tilt_weight);
        target.height_weight = settings.value("height_weight", target.height_weight);
        target.xy_kp = settings.value("xy_kp", target.xy_kp);
        target.tilt_kp = settings.value("tilt_kp", target.tilt_kp);
        target.height_kp = settings.value("height_kp", target.height_kp);
        target.max_anchor_correction_speed = settings.value(
            "max_anchor_correction_speed_mps", target.max_anchor_correction_speed);
        target.anchor_deadzone = settings.value(
            "anchor_deadzone_m", target.anchor_deadzone);
        target.max_slip_speed = settings.value(
            "max_slip_speed_mps", target.max_slip_speed);
        target.support_height = settings.value(
            "support_height_m", target.support_height);
        target.support_height_upper = settings.value(
            "support_height_upper_m", target.support_height_upper);
        target.penetration_tolerance = settings.value(
            "penetration_tolerance_m", target.penetration_tolerance);
        target.max_joint_velocity = settings.value(
            "max_joint_velocity_rps", target.max_joint_velocity);
        target.max_root_linear_velocity = settings.value(
            "max_root_linear_velocity_mps", target.max_root_linear_velocity);
        target.max_root_angular_velocity = settings.value(
            "max_root_angular_velocity_rps", target.max_root_angular_velocity);
        target.forced_support_weight_scale = settings.value(
            "forced_support_weight_scale",
            target.forced_support_weight_scale);
        target.transition_frames = settings.value(
            "transition_frames", target.transition_frames);
        target.hard_support_constraints = settings.value(
            "hard_support_constraints", target.hard_support_constraints);
    }
    const auto& settings = result.settings;
    const bool finite_positive =
        std::isfinite(settings.xy_weight) && settings.xy_weight > 0.0 &&
        std::isfinite(settings.tilt_weight) && settings.tilt_weight > 0.0 &&
        std::isfinite(settings.height_weight) && settings.height_weight > 0.0 &&
        std::isfinite(settings.xy_kp) && settings.xy_kp > 0.0 &&
        std::isfinite(settings.tilt_kp) && settings.tilt_kp > 0.0 &&
        std::isfinite(settings.height_kp) && settings.height_kp > 0.0 &&
        std::isfinite(settings.max_anchor_correction_speed) &&
        settings.max_anchor_correction_speed > 0.0 &&
        std::isfinite(settings.anchor_deadzone) &&
        settings.anchor_deadzone >= 0.0 &&
        std::isfinite(settings.max_slip_speed) && settings.max_slip_speed >= 0.0 &&
        std::isfinite(settings.support_height) && settings.support_height >= 0.0 &&
        std::isfinite(settings.support_height_upper) &&
        settings.support_height_upper >= settings.support_height &&
        std::isfinite(settings.penetration_tolerance) &&
        settings.penetration_tolerance >= 0.0 &&
        std::isfinite(settings.max_joint_velocity) &&
        settings.max_joint_velocity > 0.0 &&
        std::isfinite(settings.max_root_linear_velocity) &&
        settings.max_root_linear_velocity > 0.0 &&
        std::isfinite(settings.max_root_angular_velocity) &&
        settings.max_root_angular_velocity > 0.0 &&
        std::isfinite(settings.forced_support_weight_scale) &&
        settings.forced_support_weight_scale >= 0.0 &&
        settings.forced_support_weight_scale <= 1.0 &&
        settings.transition_frames >= 1;
    if (!finite_positive)
        throw std::runtime_error("invalid foot_contact constraint settings");
    return result;
}

}  // namespace gmr
