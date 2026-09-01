#pragma once

#include "gmr/body_map.hpp"

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace gmr {

enum class FootSide { Left, Right };

inline const char* footSideName(FootSide side) {
    return side == FootSide::Left ? "left" : "right";
}

// Points are ordered heel(-Y), heel(+Y), toe(-Y), toe(+Y).  They are
// expressed in the owning foot body's local frame and lie on the physical
// contact surface, rather than at the ankle joint origin.
struct FootSoleDefinition {
    std::string body_name;
    std::array<Eigen::Vector3d, 4> points_local{};

    Eigen::Vector3d centerLocal() const {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        for (const auto& point : points_local) center += point;
        return center / static_cast<double>(points_local.size());
    }

    Eigen::Vector3d normalLocal() const {
        const Eigen::Vector3d heel = 0.5 * (points_local[0] + points_local[1]);
        const Eigen::Vector3d toe = 0.5 * (points_local[2] + points_local[3]);
        const Eigen::Vector3d negative_y = 0.5 * (points_local[0] + points_local[2]);
        const Eigen::Vector3d positive_y = 0.5 * (points_local[1] + points_local[3]);
        Eigen::Vector3d normal = (toe - heel).cross(positive_y - negative_y);
        if (normal.norm() < 1e-9) return Eigen::Vector3d::UnitZ();
        normal.normalize();
        if (normal.dot(Eigen::Vector3d::UnitZ()) < 0.0) normal = -normal;
        return normal;
    }
};

struct FootObservation {
    FootSide side = FootSide::Left;
    std::array<Eigen::Vector3d, 4> points_world{};
    Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_world = Eigen::Vector3d::UnitZ();

    double minimumHeight() const {
        double result = std::numeric_limits<double>::infinity();
        for (const auto& point : points_world) result = std::min(result, point.z());
        return result;
    }
};

inline Eigen::Matrix3d bodyRotation(const mjData* data, int body_id) {
    Eigen::Matrix3d rotation;
    const mjtNum* value = data->xmat + 9 * body_id;
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            rotation(row, column) = value[3 * row + column];
    return rotation;
}

inline FootObservation observeFoot(const mjModel* model, const mjData* data,
                                   FootSide side,
                                   const FootSoleDefinition& definition) {
    if (!model || !data) throw std::runtime_error("observeFoot: null MuJoCo model/data");
    const int body = mj_name2id(model, mjOBJ_BODY, definition.body_name.c_str());
    if (body < 0)
        throw std::runtime_error("observeFoot: missing body " + definition.body_name);
    const Eigen::Vector3d position(data->xpos[3 * body], data->xpos[3 * body + 1],
                                   data->xpos[3 * body + 2]);
    const Eigen::Matrix3d rotation = bodyRotation(data, body);
    FootObservation observation;
    observation.side = side;
    for (size_t index = 0; index < definition.points_local.size(); ++index) {
        observation.points_world[index] =
            position + rotation * definition.points_local[index];
        observation.center_world += observation.points_world[index];
    }
    observation.center_world /= static_cast<double>(definition.points_local.size());
    observation.normal_world = (rotation * definition.normalLocal()).normalized();
    return observation;
}

// SMPL-X 的 SMP1 输入已经给出脚目标的位置和四元数，不需要再加载一份人体
// MuJoCo 模型。这里把配置中的四个局部参考点变换到世界坐标，生成与机器人 FK
// observeFoot() 相同的数据结构，从而复用同一个高度/速度/滞回接触检测器。
inline FootObservation observeFoot(const BodyMap& bodies, FootSide side,
                                   const FootSoleDefinition& definition) {
    const auto found = bodies.find(definition.body_name);
    if (found == bodies.end())
        throw std::runtime_error(
            "observeFoot: missing SMPL-X target " + definition.body_name);
    const BodyData& body = found->second;
    if (!body.position.allFinite() || !body.rot_wxyz.allFinite())
        throw std::runtime_error(
            "observeFoot: non-finite SMPL-X target " + definition.body_name);
    const double norm = body.rot_wxyz.norm();
    if (norm < 1e-9)
        throw std::runtime_error(
            "observeFoot: zero SMPL-X quaternion " + definition.body_name);
    const Eigen::Vector4d normalized = body.rot_wxyz / norm;
    const Eigen::Quaterniond quaternion(
        normalized[0], normalized[1], normalized[2], normalized[3]);
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

    FootObservation observation;
    observation.side = side;
    for (size_t index = 0; index < definition.points_local.size(); ++index) {
        observation.points_world[index] =
            body.position + rotation * definition.points_local[index];
        observation.center_world += observation.points_world[index];
    }
    observation.center_world /=
        static_cast<double>(definition.points_local.size());
    observation.normal_world =
        (rotation * definition.normalLocal()).normalized();
    return observation;
}

namespace detail {

struct SoleCandidate {
    Eigen::Vector3d local;
    double world_z = 0.0;
};

inline Eigen::Vector3d transformGeomPoint(const mjData* data, int geom,
                                          const Eigen::Vector3d& local) {
    const mjtNum* position = data->geom_xpos + 3 * geom;
    const mjtNum* rotation = data->geom_xmat + 9 * geom;
    Eigen::Vector3d world(position[0], position[1], position[2]);
    for (int row = 0; row < 3; ++row)
        world[row] += rotation[3 * row] * local.x() +
                      rotation[3 * row + 1] * local.y() +
                      rotation[3 * row + 2] * local.z();
    return world;
}

}  // namespace detail

inline std::vector<Eigen::Vector3d> footContactGeometryPointsLocal(
    const mjModel* model, const mjData* data, const std::string& body_name,
    bool contact_only = true) {
    if (!model || !data)
        throw std::runtime_error("footContactGeometryPointsLocal: null model/data");
    const int body = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body < 0)
        throw std::runtime_error(
            "footContactGeometryPointsLocal: missing body " + body_name);
    const Eigen::Vector3d body_position(
        data->xpos[3 * body], data->xpos[3 * body + 1], data->xpos[3 * body + 2]);
    const Eigen::Matrix3d body_rotation = bodyRotation(data, body);
    std::vector<Eigen::Vector3d> points;
    auto add_world = [&](const Eigen::Vector3d& world) {
        points.push_back(body_rotation.transpose() * (world - body_position));
    };
    for (int geom = 0; geom < model->ngeom; ++geom) {
        if (model->geom_bodyid[geom] != body) continue;
        if (contact_only && model->geom_contype[geom] == 0 &&
            model->geom_conaffinity[geom] == 0)
            continue;
        const int type = model->geom_type[geom];
        if (type == mjGEOM_MESH && model->geom_dataid[geom] >= 0) {
            const int mesh = model->geom_dataid[geom];
            const int begin = model->mesh_vertadr[mesh];
            const int count = model->mesh_vertnum[mesh];
            points.reserve(points.size() + static_cast<size_t>(count));
            for (int index = 0; index < count; ++index) {
                const float* vertex = model->mesh_vert + 3 * (begin + index);
                add_world(detail::transformGeomPoint(
                    data, geom, Eigen::Vector3d(vertex[0], vertex[1], vertex[2])));
            }
        } else if (type == mjGEOM_SPHERE) {
            const Eigen::Vector3d center(data->geom_xpos[3 * geom],
                                         data->geom_xpos[3 * geom + 1],
                                         data->geom_xpos[3 * geom + 2]);
            const double radius = model->geom_size[3 * geom];
            const std::array<Eigen::Vector3d, 3> axes = {
                Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
                Eigen::Vector3d::UnitZ()};
            for (const auto& axis : axes) {
                add_world(center + radius * axis);
                add_world(center - radius * axis);
            }
        } else {
            const mjtNum* bounds = model->geom_aabb + 6 * geom;
            for (int x : {-1, 1}) for (int y : {-1, 1}) for (int z : {-1, 1})
                add_world(detail::transformGeomPoint(
                    data, geom,
                    Eigen::Vector3d(bounds[0] + x * bounds[3],
                                    bounds[1] + y * bounds[4],
                                    bounds[2] + z * bounds[5])));
        }
    }
    if (points.empty())
        throw std::runtime_error("no contact geometry on foot body " + body_name);
    return points;
}

// Derive stable heel/toe contact points from compiled MuJoCo geometry.  Mesh
// vertices are transformed through geom and body poses, so mesh scale and geom
// offsets are respected.  This is intended for config generation, not the
// realtime loop.
inline FootSoleDefinition deriveFootSoleDefinition(
    const mjModel* model, const mjData* data, const std::string& body_name,
    double sole_band = 0.004, bool contact_only = true) {
    if (!model || !data)
        throw std::runtime_error("deriveFootSoleDefinition: null model/data");
    if (!std::isfinite(sole_band) || sole_band <= 0.0)
        throw std::runtime_error("sole_band must be positive and finite");
    const int body = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body < 0)
        throw std::runtime_error("deriveFootSoleDefinition: missing body " + body_name);
    const Eigen::Vector3d body_position(
        data->xpos[3 * body], data->xpos[3 * body + 1], data->xpos[3 * body + 2]);
    const Eigen::Matrix3d body_rotation = bodyRotation(data, body);
    std::vector<detail::SoleCandidate> candidates;

    auto add_world = [&](const Eigen::Vector3d& world) {
        candidates.push_back({body_rotation.transpose() * (world - body_position),
                              world.z()});
    };

    for (int geom = 0; geom < model->ngeom; ++geom) {
        if (model->geom_bodyid[geom] != body) continue;
        if (contact_only && model->geom_contype[geom] == 0 &&
            model->geom_conaffinity[geom] == 0)
            continue;
        const int type = model->geom_type[geom];
        if (type == mjGEOM_MESH && model->geom_dataid[geom] >= 0) {
            const int mesh = model->geom_dataid[geom];
            const int begin = model->mesh_vertadr[mesh];
            const int count = model->mesh_vertnum[mesh];
            for (int index = 0; index < count; ++index) {
                const float* vertex = model->mesh_vert + 3 * (begin + index);
                add_world(detail::transformGeomPoint(
                    data, geom, Eigen::Vector3d(vertex[0], vertex[1], vertex[2])));
            }
        } else if (type == mjGEOM_SPHERE) {
            const Eigen::Vector3d center(data->geom_xpos[3 * geom],
                                         data->geom_xpos[3 * geom + 1],
                                         data->geom_xpos[3 * geom + 2]);
            add_world(center - model->geom_size[3 * geom] * Eigen::Vector3d::UnitZ());
        } else {
            const mjtNum* bounds = model->geom_aabb + 6 * geom;
            for (int x : {-1, 1}) for (int y : {-1, 1}) for (int z : {-1, 1}) {
                add_world(detail::transformGeomPoint(
                    data, geom,
                    Eigen::Vector3d(bounds[0] + x * bounds[3],
                                    bounds[1] + y * bounds[4],
                                    bounds[2] + z * bounds[5])));
            }
        }
    }
    if (candidates.empty())
        throw std::runtime_error("no contact geometry on foot body " + body_name);

    const double minimum_z = std::min_element(
        candidates.begin(), candidates.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.world_z < rhs.world_z; })
                                 ->world_z;
    std::vector<detail::SoleCandidate> patch;
    for (const auto& candidate : candidates)
        if (candidate.world_z <= minimum_z + sole_band) patch.push_back(candidate);
    if (patch.size() < 4) {
        std::sort(candidates.begin(), candidates.end(),
                  [](const auto& lhs, const auto& rhs) {
                      return lhs.world_z < rhs.world_z;
                  });
        patch.assign(candidates.begin(),
                     candidates.begin() + std::min<size_t>(candidates.size(), 16));
    }
    if (patch.size() < 4)
        throw std::runtime_error("insufficient sole geometry points on " + body_name);

    double xmin = std::numeric_limits<double>::infinity();
    double xmax = -std::numeric_limits<double>::infinity();
    double ymin = std::numeric_limits<double>::infinity();
    double ymax = -std::numeric_limits<double>::infinity();
    for (const auto& candidate : patch) {
        xmin = std::min(xmin, candidate.local.x());
        xmax = std::max(xmax, candidate.local.x());
        ymin = std::min(ymin, candidate.local.y());
        ymax = std::max(ymax, candidate.local.y());
    }
    const double xscale = std::max(1e-6, xmax - xmin);
    const double yscale = std::max(1e-6, ymax - ymin);
    auto nearest = [&](double x, double y) {
        const detail::SoleCandidate* best = nullptr;
        double best_score = std::numeric_limits<double>::infinity();
        for (const auto& candidate : patch) {
            const double dx = (candidate.local.x() - x) / xscale;
            const double dy = (candidate.local.y() - y) / yscale;
            const double dz = (candidate.world_z - minimum_z) / sole_band;
            const double score = dx * dx + dy * dy + 0.05 * dz * dz;
            if (score < best_score) {
                best_score = score;
                best = &candidate;
            }
        }
        return best->local;
    };

    FootSoleDefinition definition;
    definition.body_name = body_name;
    definition.points_local = {
        nearest(xmin, ymin), nearest(xmin, ymax),
        nearest(xmax, ymin), nearest(xmax, ymax),
    };
    return definition;
}

struct FootContactDetectorConfig {
    double ground_z = 0.0;
    double enter_height = 0.025;
    double exit_height = 0.045;
    double max_enter_vertical_speed = 0.18;
    double max_exit_upward_speed = 0.30;
    double max_enter_horizontal_speed = 0.035;
    double max_exit_horizontal_speed = 0.075;
    double double_support_max_horizontal_speed = 0.025;
    int enter_frames = 2;
    int exit_frames = 1;
    int minimum_stance_frames = 2;
    bool allow_flight = false;
};

struct SourceGroundTrackerConfig {
    bool enabled = false;
    double support_track_speed = 0.04;
    double reacquire_track_speed = 0.12;
    double max_reacquire_vertical_speed = 0.30;
    double max_reacquire_horizontal_speed = 0.50;
    int reacquire_after_frames = 45;
    int stable_reacquire_frames = 5;
};

struct FootContactState {
    bool left_stance = false;
    bool right_stance = false;
    bool left_forced = false;
    bool right_forced = false;
    double left_height = std::numeric_limits<double>::infinity();
    double right_height = std::numeric_limits<double>::infinity();
    double left_vertical_speed = 0.0;
    double right_vertical_speed = 0.0;
    double left_horizontal_speed = 0.0;
    double right_horizontal_speed = 0.0;
};

// Track only the slow vertical bias of the source coordinate system.  A
// trusted stance may update the estimate, while ordinary flight freezes it.
// After an implausibly long flight, a stable lower foot can reacquire the
// floor gradually; this preserves real jumps without accepting permanent
// long-sequence root drift as airborne motion.
class SourceGroundTracker {
public:
    explicit SourceGroundTracker(SourceGroundTrackerConfig config = {})
        : config_(config) {
        if (!std::isfinite(config_.support_track_speed) ||
            config_.support_track_speed < 0.0 ||
            !std::isfinite(config_.reacquire_track_speed) ||
            config_.reacquire_track_speed < config_.support_track_speed ||
            !std::isfinite(config_.max_reacquire_vertical_speed) ||
            config_.max_reacquire_vertical_speed < 0.0 ||
            !std::isfinite(config_.max_reacquire_horizontal_speed) ||
            config_.max_reacquire_horizontal_speed < 0.0 ||
            config_.reacquire_after_frames < 1 ||
            config_.stable_reacquire_frames < 1)
            throw std::runtime_error("invalid source ground tracker configuration");
    }

    void reset() {
        initialized_ = false;
        ground_z_ = 0.0;
        initial_ground_z_ = 0.0;
        previous_timestamp_ = 0.0;
        previous_left_center_.setZero();
        previous_right_center_.setZero();
        flight_frames_ = 0;
        stable_reacquire_frames_ = 0;
        reacquiring_ = false;
    }

    double update(const FootObservation& left, const FootObservation& right,
                  const FootContactState& previous_state, double timestamp) {
        if (!std::isfinite(timestamp))
            throw std::runtime_error("source ground timestamp must be finite");
        if (!initialized_) {
            ground_z_ = std::min(left.minimumHeight(), right.minimumHeight());
            initial_ground_z_ = ground_z_;
            initialized_ = true;
            remember(left, right, timestamp);
            return ground_z_;
        }

        if (!config_.enabled) {
            remember(left, right, timestamp);
            return ground_z_;
        }

        const double elapsed = timestamp - previous_timestamp_;
        const bool valid_dt = elapsed > 0.0 && elapsed <= 0.2;
        const double dt = valid_dt ? elapsed : 0.0;
        const double left_vz = valid_dt
            ? (left.center_world.z() - previous_left_center_.z()) / dt : 0.0;
        const double right_vz = valid_dt
            ? (right.center_world.z() - previous_right_center_.z()) / dt : 0.0;
        const double left_vxy = valid_dt
            ? (left.center_world.head<2>() -
               previous_left_center_.head<2>()).norm() / dt : 0.0;
        const double right_vxy = valid_dt
            ? (right.center_world.head<2>() -
               previous_right_center_.head<2>()).norm() / dt : 0.0;

        if (previous_state.left_stance || previous_state.right_stance) {
            flight_frames_ = 0;
            stable_reacquire_frames_ = 0;
            reacquiring_ = false;
            double candidate = std::numeric_limits<double>::infinity();
            if (previous_state.left_stance)
                candidate = std::min(candidate, left.minimumHeight());
            if (previous_state.right_stance)
                candidate = std::min(candidate, right.minimumHeight());
            trackToward(candidate, config_.support_track_speed, dt);
        } else {
            ++flight_frames_;
            const bool left_lower = left.minimumHeight() <= right.minimumHeight();
            const auto& lower = left_lower ? left : right;
            const double lower_vz = left_lower ? left_vz : right_vz;
            const double lower_vxy = left_lower ? left_vxy : right_vxy;
            const bool stable = valid_dt &&
                std::abs(lower_vz) <= config_.max_reacquire_vertical_speed &&
                lower_vxy <= config_.max_reacquire_horizontal_speed;
            stable_reacquire_frames_ = stable
                ? stable_reacquire_frames_ + 1 : 0;
            if (flight_frames_ >= config_.reacquire_after_frames &&
                stable_reacquire_frames_ >= config_.stable_reacquire_frames)
                reacquiring_ = true;
            if (reacquiring_)
                trackToward(lower.minimumHeight(),
                            config_.reacquire_track_speed, dt);
        }

        remember(left, right, timestamp);
        return ground_z_;
    }

    bool initialized() const { return initialized_; }
    double groundZ() const { return ground_z_; }
    double initialGroundZ() const { return initial_ground_z_; }
    double correction() const { return ground_z_ - initial_ground_z_; }
    bool reacquiring() const { return reacquiring_; }
    int flightFrames() const { return flight_frames_; }
    const SourceGroundTrackerConfig& config() const { return config_; }

private:
    void trackToward(double candidate, double speed, double dt) {
        if (!std::isfinite(candidate) || dt <= 0.0 || speed <= 0.0) return;
        const double maximum = speed * dt;
        ground_z_ += std::clamp(candidate - ground_z_, -maximum, maximum);
    }

    void remember(const FootObservation& left, const FootObservation& right,
                  double timestamp) {
        previous_left_center_ = left.center_world;
        previous_right_center_ = right.center_world;
        previous_timestamp_ = timestamp;
    }

    SourceGroundTrackerConfig config_;
    bool initialized_ = false;
    double ground_z_ = 0.0;
    double initial_ground_z_ = 0.0;
    double previous_timestamp_ = 0.0;
    Eigen::Vector3d previous_left_center_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_right_center_ = Eigen::Vector3d::Zero();
    int flight_frames_ = 0;
    int stable_reacquire_frames_ = 0;
    bool reacquiring_ = false;
};

struct FootConstraintSettings {
    double xy_weight = 35.0;
    double tilt_weight = 60.0;
    double height_weight = 80.0;
    double xy_kp = 5.0;
    double tilt_kp = 8.0;
    double tilt_deadzone = 0.0;
    double height_kp = 10.0;
    double max_anchor_correction_speed = 0.20;
    double anchor_deadzone = 0.008;
    double max_slip_speed = 0.05;
    double support_height = 0.001;
    double support_height_upper = 0.003;
    double max_heel_toe_height_difference = 0.003;
    double max_lateral_height_difference = 0.003;
    double penetration_tolerance = 0.0;
    double mesh_floor_margin = 0.0;
    double sole_corner_floor_margin = 0.0;
    double max_joint_velocity = 8.0;
    double max_joint_acceleration = 80.0;
    double max_root_linear_velocity = 3.0;
    double max_output_root_horizontal_velocity = 0.75;
    double max_root_vertical_velocity = 0.45;
    double max_root_linear_acceleration = 3.0;
    double max_root_angular_velocity = 6.0;
    double max_root_angular_acceleration = 20.0;
    double frame_jerk_weight = 0.0;
    double forced_support_weight_scale = 0.08;
    int transition_frames = 8;
    bool hard_support_constraints = false;
};

struct FootConstraintConfig {
    bool enabled = false;
    double ground_z = 0.0;
    FootSoleDefinition left;
    FootSoleDefinition right;
    FootConstraintSettings settings;
};

struct FootConstraintDiagnostics {
    bool enabled = false;
    bool left_stance = false;
    bool right_stance = false;
    bool left_forced = false;
    bool right_forced = false;
    double left_blend = 0.0;
    double right_blend = 0.0;
    double left_slip = 0.0;
    double right_slip = 0.0;
    double left_tilt_degrees = 0.0;
    double right_tilt_degrees = 0.0;
    double left_min_height = 0.0;
    double left_max_height = 0.0;
    double left_center_height = 0.0;
    double left_heel_toe_height_difference = 0.0;
    double left_lateral_height_difference = 0.0;
    double right_min_height = 0.0;
    double right_max_height = 0.0;
    double right_center_height = 0.0;
    double right_heel_toe_height_difference = 0.0;
    double right_lateral_height_difference = 0.0;
    int daqp_exitflag = 0;
    bool relaxed_slip_constraints = false;
    int primary_support_side = -1;  // -1 none, 0 left, 1 right
    bool root_z_safety_override = false;
};

class FootContactDetector {
public:
    explicit FootContactDetector(FootContactDetectorConfig config = {})
        : config_(config) {
        if (!std::isfinite(config_.ground_z) ||
            !std::isfinite(config_.enter_height) || config_.enter_height < 0.0 ||
            !std::isfinite(config_.exit_height) ||
            config_.exit_height <= config_.enter_height ||
            !std::isfinite(config_.max_enter_vertical_speed) ||
            config_.max_enter_vertical_speed < 0.0 ||
            !std::isfinite(config_.max_exit_upward_speed) ||
            config_.max_exit_upward_speed < 0.0 || config_.enter_frames < 1 ||
            !std::isfinite(config_.max_enter_horizontal_speed) ||
            config_.max_enter_horizontal_speed < 0.0 ||
            !std::isfinite(config_.max_exit_horizontal_speed) ||
            config_.max_exit_horizontal_speed <=
                config_.max_enter_horizontal_speed ||
            !std::isfinite(config_.double_support_max_horizontal_speed) ||
            config_.double_support_max_horizontal_speed < 0.0 ||
            config_.double_support_max_horizontal_speed >=
                config_.max_exit_horizontal_speed ||
            config_.exit_frames < 1 || config_.minimum_stance_frames < 0)
            throw std::runtime_error("invalid foot contact detector configuration");
    }

    FootContactState update(const FootObservation& left,
                            const FootObservation& right, double timestamp) {
        if (!std::isfinite(timestamp))
            throw std::runtime_error("foot contact timestamp must be finite");
        double dt = 0.0;
        const bool valid_dt = has_previous_ && timestamp > previous_timestamp_ &&
                              timestamp - previous_timestamp_ <= 0.2;
        if (valid_dt) dt = timestamp - previous_timestamp_;
        const double left_vz = valid_dt
            ? (left.center_world.z() - previous_left_center_.z()) / dt : 0.0;
        const double right_vz = valid_dt
            ? (right.center_world.z() - previous_right_center_.z()) / dt : 0.0;
        const double left_vxy = valid_dt
            ? (left.center_world.head<2>() -
               previous_left_center_.head<2>()).norm() / dt : 0.0;
        const double right_vxy = valid_dt
            ? (right.center_world.head<2>() -
               previous_right_center_.head<2>()).norm() / dt : 0.0;

        updateLatch(left_latch_, left.minimumHeight(), left_vz, left_vxy);
        updateLatch(right_latch_, right.minimumHeight(), right_vz, right_vxy);

        FootContactState result;
        result.left_stance = left_latch_.active;
        result.right_stance = right_latch_.active;
        result.left_height = left.minimumHeight() - config_.ground_z;
        result.right_height = right.minimumHeight() - config_.ground_z;
        result.left_vertical_speed = left_vz;
        result.right_vertical_speed = right_vz;
        result.left_horizontal_speed = left_vxy;
        result.right_horizontal_speed = right_vxy;

        // Pin both feet only when both are genuinely stationary.  During
        // walking and turning, a transient low double-support pose otherwise
        // creates conflicting world anchors that are absorbed by pelvis/waist
        // sway.  Keep the slower foot as primary support and release the
        // faster one.  If neither is stationary, keep only a weak grounding
        // hint on the lower/slower foot.
        if (result.left_stance && result.right_stance &&
            std::max(left_vxy, right_vxy) >
                config_.double_support_max_horizontal_speed) {
            const bool left_stationary =
                left_vxy <= config_.double_support_max_horizontal_speed;
            const bool right_stationary =
                right_vxy <= config_.double_support_max_horizontal_speed;
            if (left_stationary && !right_stationary) {
                result.right_stance = false;
                ambiguous_support_side_valid_ = false;
            } else if (right_stationary && !left_stationary) {
                result.left_stance = false;
                ambiguous_support_side_valid_ = false;
            } else {
                if (!ambiguous_support_side_valid_) {
                    const double left_score =
                        result.left_height + 0.05 * left_vxy;
                    const double right_score =
                        result.right_height + 0.05 * right_vxy;
                    ambiguous_support_side_ = left_score <= right_score
                        ? FootSide::Left : FootSide::Right;
                    ambiguous_support_side_valid_ = true;
                }
                if (ambiguous_support_side_ == FootSide::Left) {
                    result.right_stance = false;
                    result.left_forced = true;
                } else {
                    result.left_stance = false;
                    result.right_forced = true;
                }
            }
        } else {
            ambiguous_support_side_valid_ = false;
        }

        // A low but moving single support is a sliding/turning ground hint.
        // Keep its sole flat and seated, but mark it forced so the solver does
        // not create a conflicting world-XY anchor.
        if (result.left_stance && !result.right_stance &&
            left_vxy > config_.double_support_max_horizontal_speed)
            result.left_forced = true;
        if (result.right_stance && !result.left_stance &&
            right_vxy > config_.double_support_max_horizontal_speed)
            result.right_forced = true;

        if (!config_.allow_flight && !result.left_stance && !result.right_stance) {
            if (!forced_side_valid_)
                forced_side_ = result.left_height <= result.right_height
                    ? FootSide::Left : FootSide::Right;
            forced_side_valid_ = true;
            if (forced_side_ == FootSide::Left) {
                result.left_stance = true;
                result.left_forced = true;
            } else {
                result.right_stance = true;
                result.right_forced = true;
            }
        } else if (result.left_stance || result.right_stance) {
            forced_side_valid_ = false;
        }

        previous_left_center_ = left.center_world;
        previous_right_center_ = right.center_world;
        previous_timestamp_ = timestamp;
        has_previous_ = true;
        return result;
    }

    void reset() {
        left_latch_ = {};
        right_latch_ = {};
        has_previous_ = false;
        forced_side_valid_ = false;
        ambiguous_support_side_valid_ = false;
    }

    const FootContactDetectorConfig& config() const { return config_; }

private:
    struct Latch {
        bool active = false;
        int enter_count = 0;
        int exit_count = 0;
        int stance_frames = 0;
    };

    void updateLatch(Latch& latch, double minimum_z, double vertical_speed,
                     double horizontal_speed) {
        const double height = minimum_z - config_.ground_z;
        if (latch.active) {
            ++latch.stance_frames;
            const bool leaving =
                height >= config_.exit_height ||
                vertical_speed >= config_.max_exit_upward_speed ||
                horizontal_speed >= config_.max_exit_horizontal_speed;
            latch.exit_count = leaving ? latch.exit_count + 1 : 0;
            if (latch.stance_frames >= config_.minimum_stance_frames &&
                latch.exit_count >= config_.exit_frames) {
                latch = {};
            }
        } else {
            const bool entering =
                height <= config_.enter_height &&
                std::abs(vertical_speed) <= config_.max_enter_vertical_speed &&
                horizontal_speed <= config_.max_enter_horizontal_speed;
            latch.enter_count = entering ? latch.enter_count + 1 : 0;
            if (latch.enter_count >= config_.enter_frames) {
                latch.active = true;
                latch.enter_count = 0;
                latch.stance_frames = 0;
            }
        }
    }

    FootContactDetectorConfig config_;
    Latch left_latch_;
    Latch right_latch_;
    bool has_previous_ = false;
    double previous_timestamp_ = 0.0;
    Eigen::Vector3d previous_left_center_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_right_center_ = Eigen::Vector3d::Zero();
    bool forced_side_valid_ = false;
    FootSide forced_side_ = FootSide::Left;
    bool ambiguous_support_side_valid_ = false;
    FootSide ambiguous_support_side_ = FootSide::Left;
};

}  // namespace gmr
