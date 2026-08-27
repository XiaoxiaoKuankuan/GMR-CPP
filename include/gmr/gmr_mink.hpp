#pragma once
/**
 * gmr_mink.hpp — C++ port of Python GMR + mink IK
 */
#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

extern "C" {
#include "third_party/daqp/include/api.h"
#include "third_party/daqp/include/types.h"
}

#include "gmr/body_map.hpp"
#include "gmr/foot_contact_json.hpp"
#include "gmr/realtime_motion_guard.hpp"
#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <iostream>
#include <algorithm>
#include <limits>
#include <chrono>
#include <memory>

namespace gmr_mink {

using BodyData = gmr::BodyData;
using BodyMap  = gmr::BodyMap;

namespace quat {
inline Eigen::Vector4d normalise(const Eigen::Vector4d& q) {
    double n = q.norm(); return n < 1e-12 ? Eigen::Vector4d(1,0,0,0) : q/n;
}
inline Eigen::Vector4d conjugate(const Eigen::Vector4d& q) {
    return {q[0],-q[1],-q[2],-q[3]};
}
inline Eigen::Vector4d multiply(const Eigen::Vector4d& a, const Eigen::Vector4d& b) {
    return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
            a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
            a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
inline Eigen::Vector3d rotate(const Eigen::Vector4d& q, const Eigen::Vector3d& v) {
    Eigen::Vector4d qv{0,v.x(),v.y(),v.z()};
    auto r = multiply(multiply(q,qv),conjugate(q));
    return {r[1],r[2],r[3]};
}
inline Eigen::Vector3d so3_error(const Eigen::Vector4d& q_cur, const Eigen::Vector4d& q_tgt) {
    auto qr = normalise(multiply(conjugate(q_cur), q_tgt));
    if (qr[0] < 0) qr = -qr;
    double w = std::clamp(qr[0], -1.0, 1.0);
    double angle = 2.0 * std::acos(w);
    double s = std::sqrt(std::max(1e-12, 1.0-w*w));
    return Eigen::Vector3d{qr[1]/s, qr[2]/s, qr[3]/s} * angle;
}
} // namespace quat

struct IKEntry {
    std::string     robot_body;
    std::string     human_body;
    double          pos_weight = 0.0;
    double          rot_weight = 0.0;
    Eigen::Vector3d pos_offset = Eigen::Vector3d::Zero();
    Eigen::Vector4d rot_offset = Eigen::Vector4d(1,0,0,0);
};

struct MotionPreservingConfig {
    bool configured = false;
    std::string root_body = "base_link";
    std::string waist_body = "waist_yaw_link";
    Eigen::Vector3d root_position_axes = Eigen::Vector3d(1.0, 1.0, 0.0);
    Eigen::Vector3d root_rotation_axes = Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d waist_rotation_axes = Eigen::Vector3d(0.0, 0.0, 1.0);
    double root_roll_upright_weight = 20.0;
    double torso_pitch_weight_scale = 1.0;
    double joint_velocity_weight = 0.05;
    double joint_acceleration_weight = 0.20;
};

struct FrameTemporalLimitDiagnostics {
    double timestep = 0.0;
    int bounded_dofs = 0;
    int active_bound_dofs = 0;
    double maximum_bound_overrun = 0.0;
    uint64_t total_active_bound_dofs = 0;
};

class GMR {
public:
    GMR(const std::string& xml_path,
        const std::string& ik_config_path,
        double actual_human_height = 1.8,
        double damping = 0.5,
        bool   verbose = false)
        : damping_(damping), verbose_(verbose)
    {
        char err[1024] = {};
        model_ = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
        if (!model_) throw std::runtime_error(std::string("mj_loadXML: ")+err);
        data_ = mj_makeData(model_);
        nv_ = model_->nv;
        nq_ = model_->nq;

        std::ifstream f(ik_config_path);
        if (!f) throw std::runtime_error("Cannot open: "+ik_config_path);
        nlohmann::json j; f >> j;

        foot_config_ = gmr::footConstraintConfigFromJson(j);
        foot_contact_enabled_ = foot_config_.enabled;

        if (j.contains("motion_preserving")) {
            const auto& value = j.at("motion_preserving");
            motion_config_.configured = value.value("enabled", false);
            motion_config_.root_body = value.value(
                "root_body", motion_config_.root_body);
            motion_config_.waist_body = value.value(
                "waist_body", motion_config_.waist_body);
            auto parse_axes = [&](const char* key, Eigen::Vector3d fallback) {
                if (!value.contains(key)) return fallback;
                const auto& axes = value.at(key);
                if (!axes.is_array() || axes.size() != 3)
                    throw std::runtime_error(
                        std::string("motion_preserving.") + key +
                        " must contain three axis weights");
                Eigen::Vector3d result(
                    axes.at(0).get<double>(), axes.at(1).get<double>(),
                    axes.at(2).get<double>());
                if (!result.allFinite() || (result.array() < 0.0).any() ||
                    (result.array() > 1.0).any())
                    throw std::runtime_error(
                        std::string("motion_preserving.") + key +
                        " must be finite and in [0,1]");
                return result;
            };
            motion_config_.root_position_axes = parse_axes(
                "root_position_axes", motion_config_.root_position_axes);
            motion_config_.root_rotation_axes = parse_axes(
                "root_rotation_axes", motion_config_.root_rotation_axes);
            motion_config_.waist_rotation_axes = parse_axes(
                "waist_rotation_axes", motion_config_.waist_rotation_axes);
            motion_config_.root_roll_upright_weight = value.value(
                "root_roll_upright_weight",
                value.value("root_upright_weight",
                            motion_config_.root_roll_upright_weight));
            motion_config_.torso_pitch_weight_scale = value.value(
                "torso_pitch_weight_scale",
                motion_config_.torso_pitch_weight_scale);
            motion_config_.joint_velocity_weight = value.value(
                "joint_velocity_weight", motion_config_.joint_velocity_weight);
            motion_config_.joint_acceleration_weight = value.value(
                "joint_acceleration_weight",
                motion_config_.joint_acceleration_weight);
            if (motion_config_.root_body.empty() ||
                motion_config_.waist_body.empty() ||
                !std::isfinite(motion_config_.root_roll_upright_weight) ||
                motion_config_.root_roll_upright_weight < 0.0 ||
                !std::isfinite(motion_config_.torso_pitch_weight_scale) ||
                motion_config_.torso_pitch_weight_scale < 0.0 ||
                motion_config_.torso_pitch_weight_scale > 5.0 ||
                !std::isfinite(motion_config_.joint_velocity_weight) ||
                motion_config_.joint_velocity_weight < 0.0 ||
                !std::isfinite(motion_config_.joint_acceleration_weight) ||
                motion_config_.joint_acceleration_weight < 0.0)
                throw std::runtime_error(
                    "invalid motion_preserving configuration");
        }

        human_root_name_ = j["human_root_name"].get<std::string>();
        ground_height_   = j["ground_height"].get<double>();
        double ratio     = actual_human_height / j["human_height_assumption"].get<double>();
        for (auto& [k,v] : j["human_scale_table"].items())
            human_scale_table_[k] = v.get<double>() * ratio;

        use_table1_ = j.value("use_ik_match_table1", false);
        use_table2_ = j.value("use_ik_match_table2", false);

        auto parse = [&](const std::string& key,
                         std::vector<IKEntry>& entries,
                         std::map<std::string,Eigen::Vector3d>& pos_off,
                         std::map<std::string,Eigen::Vector4d>& rot_off) {
            if (!j.contains(key)) return;
            for (auto& [frame_name, entry] : j[key].items()) {
                IKEntry e;
                e.robot_body = frame_name;
                e.human_body = entry[0].get<std::string>();
                e.pos_weight = entry[1].get<double>();
                e.rot_weight = entry[2].get<double>();
                e.pos_offset = {entry[3][0].get<double>(),
                                entry[3][1].get<double>(),
                                entry[3][2].get<double>()};
                e.rot_offset = {entry[4][0].get<double>(),
                                entry[4][1].get<double>(),
                                entry[4][2].get<double>(),
                                entry[4][3].get<double>()};
                pos_off[e.human_body] = e.pos_offset - Eigen::Vector3d(0,0,ground_height_);
                rot_off[e.human_body] = e.rot_offset;
                if (e.pos_weight != 0 || e.rot_weight != 0)
                    entries.push_back(e);
            }
        };
        parse("ik_match_table1", entries1_, pos_offsets1_, rot_offsets1_);
        parse("ik_match_table2", entries2_, pos_offsets2_, rot_offsets2_);

        buildVelBounds();

        if (verbose_)
            std::cout << "[GMR] nq=" << nq_ << " nv=" << nv_
                      << " vel_bounds=" << vel_bounds_.size()
                      << " t1=" << entries1_.size()
                      << " t2=" << entries2_.size() << "\n";

        mj_resetData(model_, data_);
        mj_forward(model_, data_);
        if (foot_config_.enabled) {
            for (const auto& definition : {foot_config_.left, foot_config_.right}) {
                if (mj_name2id(model_, mjOBJ_BODY,
                               definition.body_name.c_str()) < 0)
                    throw std::runtime_error(
                        "foot_contact target body is missing: " +
                        definition.body_name);
            }
            left_guard_points_ = gmr::footContactGeometryPointsLocal(
                model_, data_, foot_config_.left.body_name);
            right_guard_points_ = gmr::footContactGeometryPointsLocal(
                model_, data_, foot_config_.right.body_name);
        }
        if (motion_config_.configured) {
            for (const auto& body : {
                     motion_config_.root_body, motion_config_.waist_body})
                if (mj_name2id(model_, mjOBJ_BODY, body.c_str()) < 0)
                    throw std::runtime_error(
                        "motion_preserving target body is missing: " + body);
        }
    }

    ~GMR() {
        if (data_)  mj_deleteData(data_);
        if (model_) mj_deleteModel(model_);
    }
    GMR(const GMR&) = delete;
    GMR& operator=(const GMR&) = delete;

    Eigen::VectorXd retarget(const BodyMap& human_data, bool offset_to_ground = false)
    {
        if (frame_temporal_limits_enabled_) beginFrameTemporalLimits();
        try {
        BodyMap scaled = scaleHumanData(human_data);
        BodyMap offset = offsetHumanData(scaled, pos_offsets1_, rot_offsets1_);
        offset = applyGroundOffset(offset);
        if (offset_to_ground) offset = offsetToGround(offset);
        scaled_human_data_ = offset;

        if (frame_count_ == 0) {
            auto it = offset.find(human_root_name_);
            if (it != offset.end()) {
                const Eigen::Vector4d& hq = it->second.rot_wxyz;
                double n = hq.norm();
                Eigen::Vector4d q = (n > 1e-6) ? hq/n : Eigen::Vector4d(1,0,0,0);
                double siny = 2.0*(q[0]*q[3] + q[1]*q[2]);
                double cosy = 1.0 - 2.0*(q[2]*q[2] + q[3]*q[3]);
                double yaw  = std::atan2(siny, cosy);
                data_->qpos[3] = std::cos(yaw*0.5);
                data_->qpos[4] = 0.0;
                data_->qpos[5] = 0.0;
                data_->qpos[6] = std::sin(yaw*0.5);
                mj_forward(model_, data_);
            }
        }
        frame_count_++;
        // With a shared per-frame budget, table 1 must already see contact.
        // Otherwise it can spend the whole root/leg allowance on an infeasible
        // pose and leave table 2 no legal motion with which to recover the sole.
        if (use_table1_)
            solveIK(offset, entries1_,
                    !use_table2_ || frame_temporal_limits_active_);
        if (use_table2_) solveIK(offset, entries2_, true);
        if (motion_preserving_enabled_) applyJointTemporalRegularization();
        if (realtime_motion_guard_) {
            Eigen::Map<Eigen::VectorXd> configuration(data_->qpos, nq_);
            realtime_motion_guard_->apply(
                configuration, realtime_input_timestamp_);
            clampJoints();
            mj_forward(model_, data_);
        }
        // The batch temporal path keeps root/leg/contact limits in the same QP.
        // A direct root-Z projection here would bypass that shared frame budget
        // and recreate the one-frame vertical jump the QP is meant to prevent.
        if (contactConstraintsActive() && !frame_temporal_limits_active_)
            projectContactConfiguration(true);

        updateFootDiagnostics();

        if (frame_temporal_limits_active_) finishFrameTemporalLimits();

        Eigen::VectorXd out(nq_);
        for (int i = 0; i < nq_; ++i) out[i] = data_->qpos[i];
        return out;
        } catch (...) {
            frame_temporal_limits_active_ = false;
            throw;
        }
    }

    const BodyMap& getScaledHumanData() const { return scaled_human_data_; }
    bool footContactConfigured() const { return foot_config_.enabled; }
    bool footContactEnabled() const { return foot_contact_enabled_; }
    bool footContactsInitialized() const { return foot_contacts_initialized_; }
    const gmr::FootConstraintDiagnostics& footContactDiagnostics() const {
        return foot_diagnostics_;
    }
    void setFootContactEnabled(bool enabled) {
        if (enabled && !foot_config_.enabled)
            throw std::runtime_error(
                "foot contact constraints requested but config has no enabled foot_contact block");
        foot_contact_enabled_ = enabled;
        if (!enabled) {
            foot_contacts_initialized_ = false;
            left_contact_ = {};
            right_contact_ = {};
            foot_diagnostics_ = {};
            primary_support_side_ = -1;
        }
    }
    void setFootContactWeightScale(double scale) {
        if (!std::isfinite(scale) || scale < 0.0 || scale > 5.0)
            throw std::runtime_error(
                "foot contact weight scale must be finite and in [0, 5]");
        foot_contact_weight_scale_ = scale;
    }
    bool motionPreservingConfigured() const {
        return motion_config_.configured;
    }
    bool motionPreservingEnabled() const {
        return motion_preserving_enabled_;
    }
    void setMotionPreservingEnabled(bool enabled) {
        if (enabled && !motion_config_.configured)
            throw std::runtime_error(
                "motion-preserving retargeting requested but the config has no enabled profile");
        motion_preserving_enabled_ = enabled;
        primary_support_side_ = -1;
        resetTemporalState();
    }
    void setReferenceTimestep(double seconds) {
        if (!std::isfinite(seconds) || seconds <= 0.0 || seconds > 0.2)
            throw std::runtime_error(
                "reference timestep must be finite and in (0,0.2]");
        reference_timestep_ = seconds;
    }
    void setTemporalRegularizationWeightScale(double scale) {
        if (!std::isfinite(scale) || scale < 0.0 || scale > 5.0)
            throw std::runtime_error(
                "temporal regularization weight scale must be finite and in [0,5]");
        temporal_regularization_weight_scale_ = scale;
    }
    void setConfiguration(const Eigen::VectorXd& qpos) {
        if (qpos.size() != nq_ || !qpos.allFinite())
            throw std::runtime_error("GMR configuration size/finite check failed");
        for (int index = 0; index < nq_; ++index) data_->qpos[index] = qpos[index];
        mj_normalizeQuat(model_, data_->qpos);
        clampJoints();
        mj_forward(model_, data_);
        resetTemporalStateFromCurrent();
        if (frame_temporal_limits_enabled_)
            resetFrameTemporalLimitsFromCurrent();
    }
    void initializeFootContacts(const gmr::FootContactState& state) {
        if (!foot_contact_enabled_) return;
        foot_contacts_initialized_ = true;
        initializeFootRuntime(left_contact_, state.left_stance,
                              state.left_forced, foot_config_.left,
                              gmr::FootSide::Left);
        initializeFootRuntime(right_contact_, state.right_stance,
                              state.right_forced, foot_config_.right,
                              gmr::FootSide::Right);
        updateFootDiagnostics();
    }
    void settleFootContacts(int soft_iterations = 30,
                            int hard_iterations = 5) {
        if (!contactConstraintsActive()) return;
        if (soft_iterations < 0 || hard_iterations < 0)
            throw std::runtime_error(
                "foot-contact settle iterations must be non-negative");
        const bool hard_constraints =
            foot_config_.settings.hard_support_constraints;
        const BodyMap no_targets;
        const std::vector<IKEntry> no_entries;
        try {
            // Remove the competing human pose tasks while a newly initialized
            // stance sole is brought onto the plane.  Once it is flat, the
            // normal IK can preserve that feasible state with hard bands.
            foot_config_.settings.hard_support_constraints = false;
            for (int iteration = 0; iteration < soft_iterations; ++iteration)
                runIKStep(no_targets, no_entries, true);
            // The contact-only projection may move a newly touching sole by a
            // few millimetres.  Re-capture its feasible anchor and plane error
            // before hard constraints are restored, otherwise the transition
            // corridor would still be based on the pre-projection tilted foot.
            if (left_contact_.requested)
                captureFootAnchor(
                    left_contact_, foot_config_.left, gmr::FootSide::Left);
            if (right_contact_.requested)
                captureFootAnchor(
                    right_contact_, foot_config_.right, gmr::FootSide::Right);
            foot_config_.settings.hard_support_constraints = hard_constraints;
            for (int iteration = 0; iteration < hard_iterations; ++iteration)
                runIKStep(no_targets, no_entries, true);
        } catch (...) {
            foot_config_.settings.hard_support_constraints = hard_constraints;
            throw;
        }
        projectContactConfiguration(true);
        updateFootDiagnostics();
    }
    void setFootContactState(const gmr::FootContactState& state,
                             bool advance_transition = true) {
        if (!foot_contact_enabled_) return;
        updateFootRuntime(left_contact_, state.left_stance,
                          state.left_forced, foot_config_.left,
                          gmr::FootSide::Left, advance_transition);
        updateFootRuntime(right_contact_, state.right_stance,
                          state.right_forced, foot_config_.right,
                          gmr::FootSide::Right, advance_transition);
    }
    void setGroundOffset(double offset) {
        if (!std::isfinite(offset))
            throw std::runtime_error("ground offset must be finite");
        ground_offset_ = offset;
    }
    void setGroundClearance(double clearance) {
        if (!std::isfinite(clearance) || clearance < 0.0)
            throw std::runtime_error(
                "ground clearance must be finite and >= 0");
        ground_clearance_ = clearance;
    }
    double calibrateGroundOffset(const BodyMap& human_data, double clearance) {
        if (!std::isfinite(clearance) || clearance < 0.0)
            throw std::runtime_error(
                "ground calibration clearance must be finite and >= 0");
        const BodyMap scaled = scaleHumanData(human_data);
        const BodyMap offset = offsetHumanData(
            scaled, pos_offsets1_, rot_offsets1_);
        double lowest = std::numeric_limits<double>::infinity();
        for (const auto& [name, body] : offset) {
            std::string lower = name;
            std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
            if (lower.find("foot") != std::string::npos)
                lowest = std::min(lowest, body.position.z());
        }
        if (!std::isfinite(lowest))
            throw std::runtime_error(
                "ground calibration requires at least one human foot target");
        ground_offset_ = lowest - clearance;
        return ground_offset_;
    }
    void enableRealtimeMotionGuard(
        const gmr::RealtimeMotionGuardConfig& config) {
        realtime_motion_guard_ =
            std::make_unique<gmr::RealtimeMotionGuard>(model_, config);
        Eigen::Map<const Eigen::VectorXd> configuration(data_->qpos, nq_);
        realtime_motion_guard_->reset(configuration);
    }
    void disableRealtimeMotionGuard() { realtime_motion_guard_.reset(); }
    bool realtimeMotionGuardEnabled() const {
        return static_cast<bool>(realtime_motion_guard_);
    }
    void setRealtimeInputTimestamp(double timestamp) {
        if (!std::isfinite(timestamp))
            throw std::runtime_error("realtime input timestamp must be finite");
        realtime_input_timestamp_ = timestamp;
    }
    const gmr::RealtimeMotionGuardDiagnostics& realtimeMotionGuardDiagnostics()
        const {
        if (!realtime_motion_guard_)
            throw std::runtime_error("realtime motion guard is not enabled");
        return realtime_motion_guard_->diagnostics();
    }
    void enableFrameTemporalLimits(
        const gmr::RealtimeMotionGuardConfig& config) {
        frame_temporal_guard_config_ = config;
        buildFrameTemporalDofLimits();
        frame_temporal_limits_enabled_ = true;
        resetFrameTemporalLimitsFromCurrent();
    }
    void disableFrameTemporalLimits() {
        frame_temporal_limits_enabled_ = false;
        frame_temporal_limits_active_ = false;
        frame_temporal_initialized_ = false;
        frame_temporal_start_qpos_.resize(0);
        previous_frame_velocity_.resize(0);
        frame_displacement_lower_.resize(0);
        frame_displacement_upper_.resize(0);
        frame_temporal_diagnostics_ = {};
    }
    bool frameTemporalLimitsEnabled() const {
        return frame_temporal_limits_enabled_;
    }
    const FrameTemporalLimitDiagnostics& frameTemporalLimitDiagnostics() const {
        return frame_temporal_diagnostics_;
    }

private:
    mjModel* model_ = nullptr;
    mjData*  data_  = nullptr;
    int nv_, nq_;
    double damping_;
    bool   verbose_;

    std::string human_root_name_;
    double ground_height_ = 0.0;
    double ground_offset_ = 0.0;
    double ground_clearance_ = 0.06;
    bool use_table1_ = false, use_table2_ = false;

    std::map<std::string,double>          human_scale_table_;
    std::map<std::string,Eigen::Vector3d> pos_offsets1_, pos_offsets2_;
    std::map<std::string,Eigen::Vector4d> rot_offsets1_, rot_offsets2_;
    std::vector<IKEntry> entries1_, entries2_;
    BodyMap scaled_human_data_;

    int frame_count_ = 0;

    struct FootRuntime {
        bool requested = false;
        bool forced = false;
        bool anchor_valid = false;
        bool transition_pending_solve = false;
        double blend = 0.0;
        Eigen::Vector2d anchor_xy = Eigen::Vector2d::Zero();
        double entry_center_height = 0.0;
        double entry_max_height = 0.0;
        double entry_heel_toe_height_difference = 0.0;
        double entry_lateral_height_difference = 0.0;
    };

    struct GeneralConstraint {
        Eigen::VectorXd row;
        double lower = -1e30;
        double upper = 1e30;
        bool relaxable = false;
    };

    gmr::FootConstraintConfig foot_config_;
    bool foot_contact_enabled_ = false;
    double foot_contact_weight_scale_ = 1.0;
    bool foot_contacts_initialized_ = false;
    FootRuntime left_contact_;
    FootRuntime right_contact_;
    gmr::FootConstraintDiagnostics foot_diagnostics_;
    std::vector<Eigen::Vector3d> left_guard_points_;
    std::vector<Eigen::Vector3d> right_guard_points_;
    int last_contact_daqp_exitflag_ = 0;
    bool last_contact_relaxed_ = false;
    bool last_root_z_safety_override_ = false;

    MotionPreservingConfig motion_config_;
    bool motion_preserving_enabled_ = false;
    int primary_support_side_ = -1;  // -1 none, 0 left, 1 right
    double reference_timestep_ = 0.02;
    double previous_reference_timestep_ = 0.02;
    double temporal_regularization_weight_scale_ = 1.0;
    bool temporal_state_initialized_ = false;
    Eigen::VectorXd previous_temporal_qpos_;
    Eigen::VectorXd previous_previous_temporal_qpos_;
    std::unique_ptr<gmr::RealtimeMotionGuard> realtime_motion_guard_;
    double realtime_input_timestamp_ =
        std::numeric_limits<double>::quiet_NaN();

    struct FrameTemporalDofLimit {
        double velocity = std::numeric_limits<double>::infinity();
        double acceleration = std::numeric_limits<double>::infinity();
    };
    bool frame_temporal_limits_enabled_ = false;
    bool frame_temporal_limits_active_ = false;
    bool frame_temporal_initialized_ = false;
    double frame_temporal_timestep_ = 0.02;
    gmr::RealtimeMotionGuardConfig frame_temporal_guard_config_;
    std::vector<FrameTemporalDofLimit> frame_temporal_dof_limits_;
    Eigen::VectorXd frame_temporal_start_qpos_;
    Eigen::VectorXd previous_frame_velocity_;
    Eigen::VectorXd frame_displacement_lower_;
    Eigen::VectorXd frame_displacement_upper_;
    FrameTemporalLimitDiagnostics frame_temporal_diagnostics_;

    struct VelBound { int vadr; int qadr; double lo; double hi; };
    std::vector<VelBound> vel_bounds_;

    void buildVelBounds() {
        vel_bounds_.clear();
        for (int i = 0; i < model_->njnt; ++i) {
            if (!model_->jnt_limited[i]) continue;
            int jtype = model_->jnt_type[i];
            if (jtype != mjJNT_HINGE && jtype != mjJNT_SLIDE) continue;
            VelBound b;
            b.qadr = model_->jnt_qposadr[i];
            b.vadr = model_->jnt_dofadr[i];
            b.lo   = model_->jnt_range[i*2+0];
            b.hi   = model_->jnt_range[i*2+1];
            vel_bounds_.push_back(b);
        }
    }

    void buildFrameTemporalDofLimits() {
        const auto finite_positive = [](double value) {
            return std::isfinite(value) && value > 0.0;
        };
        const auto& guard = frame_temporal_guard_config_;
        const auto& contact = foot_config_.settings;
        if (!finite_positive(guard.max_joint_velocity) ||
            !finite_positive(guard.max_joint_acceleration) ||
            !finite_positive(guard.max_arm_velocity) ||
            !finite_positive(guard.max_arm_acceleration) ||
            !finite_positive(guard.minimum_timestep) ||
            !finite_positive(guard.maximum_timestep) ||
            guard.minimum_timestep > guard.maximum_timestep ||
            !finite_positive(contact.max_output_root_horizontal_velocity) ||
            !finite_positive(contact.max_root_vertical_velocity) ||
            !finite_positive(contact.max_root_linear_acceleration) ||
            !finite_positive(contact.max_root_angular_velocity) ||
            !finite_positive(contact.max_root_angular_acceleration))
            throw std::runtime_error(
                "invalid frame temporal-limit configuration");

        frame_temporal_dof_limits_.assign(nv_, {});
        const auto contains = [](const std::vector<std::string>& names,
                                 const std::string& name) {
            return std::find(names.begin(), names.end(), name) != names.end();
        };
        for (int joint = 0; joint < model_->njnt; ++joint) {
            const int type = model_->jnt_type[joint];
            const int dof = model_->jnt_dofadr[joint];
            if (type == mjJNT_HINGE || type == mjJNT_SLIDE) {
                const char* raw_name =
                    mj_id2name(model_, mjOBJ_JOINT, joint);
                const std::string name = raw_name ? raw_name : "";
                const bool arm = contains(guard.left_arm_joints, name) ||
                                 contains(guard.right_arm_joints, name);
                double velocity = arm ? guard.max_arm_velocity :
                                        guard.max_joint_velocity;
                const auto configured = guard.joint_velocity_limits.find(name);
                if (configured != guard.joint_velocity_limits.end())
                    velocity = configured->second;
                // Realtime safety records the actuator ceiling, while the
                // contact profile may choose a deliberately slower batch
                // tracking limit.  The shared QP always takes the safer one.
                velocity = std::min(
                    velocity, contact.max_joint_velocity);
                if (!finite_positive(velocity))
                    throw std::runtime_error(
                        "invalid frame velocity limit for joint " + name);
                frame_temporal_dof_limits_[dof] = {
                    velocity,
                    arm ? guard.max_arm_acceleration :
                          guard.max_joint_acceleration};
            } else if (type == mjJNT_FREE) {
                const double horizontal_component =
                    contact.max_output_root_horizontal_velocity /
                    std::sqrt(2.0);
                const double linear_acceleration_component =
                    contact.max_root_linear_acceleration /
                    std::sqrt(3.0);
                const double angular_velocity_component =
                    contact.max_root_angular_velocity /
                    std::sqrt(3.0);
                const double angular_acceleration_component =
                    contact.max_root_angular_acceleration /
                    std::sqrt(3.0);
                frame_temporal_dof_limits_[dof + 0] = {
                    horizontal_component,
                    linear_acceleration_component};
                frame_temporal_dof_limits_[dof + 1] = {
                    horizontal_component,
                    linear_acceleration_component};
                frame_temporal_dof_limits_[dof + 2] = {
                    contact.max_root_vertical_velocity,
                    linear_acceleration_component};
                for (int axis = 3; axis < 6; ++axis)
                    frame_temporal_dof_limits_[dof + axis] = {
                        angular_velocity_component,
                        angular_acceleration_component};
            }
        }
    }

    void resetFrameTemporalLimitsFromCurrent() {
        frame_temporal_limits_active_ = false;
        frame_temporal_initialized_ = true;
        frame_temporal_start_qpos_.resize(nq_);
        for (int index = 0; index < nq_; ++index)
            frame_temporal_start_qpos_[index] = data_->qpos[index];
        previous_frame_velocity_ = Eigen::VectorXd::Zero(nv_);
        frame_displacement_lower_ = Eigen::VectorXd::Constant(
            nv_, -std::numeric_limits<double>::infinity());
        frame_displacement_upper_ = Eigen::VectorXd::Constant(
            nv_, std::numeric_limits<double>::infinity());
        frame_temporal_diagnostics_ = {};
    }

    void beginFrameTemporalLimits() {
        if (frame_temporal_limits_active_)
            throw std::runtime_error(
                "frame temporal limits cannot begin twice");
        if (!frame_temporal_initialized_)
            resetFrameTemporalLimitsFromCurrent();
        frame_temporal_timestep_ = std::clamp(
            reference_timestep_,
            frame_temporal_guard_config_.minimum_timestep,
            frame_temporal_guard_config_.maximum_timestep);
        for (int index = 0; index < nq_; ++index)
            frame_temporal_start_qpos_[index] = data_->qpos[index];

        frame_temporal_diagnostics_.timestep = frame_temporal_timestep_;
        frame_temporal_diagnostics_.bounded_dofs = 0;
        frame_temporal_diagnostics_.active_bound_dofs = 0;
        frame_temporal_diagnostics_.maximum_bound_overrun = 0.0;
        for (int dof = 0; dof < nv_; ++dof) {
            const auto& limit = frame_temporal_dof_limits_[dof];
            if (!std::isfinite(limit.velocity) ||
                !std::isfinite(limit.acceleration)) {
                frame_displacement_lower_[dof] =
                    -std::numeric_limits<double>::infinity();
                frame_displacement_upper_[dof] =
                    std::numeric_limits<double>::infinity();
                continue;
            }
            ++frame_temporal_diagnostics_.bounded_dofs;
            const double acceleration_step =
                limit.acceleration * frame_temporal_timestep_;
            double lower_velocity = std::max(
                -limit.velocity,
                previous_frame_velocity_[dof] - acceleration_step);
            double upper_velocity = std::min(
                limit.velocity,
                previous_frame_velocity_[dof] + acceleration_step);
            // A contact emergency may require an immediate stop even when the
            // nominal acceleration corridor says the joint should keep moving.
            // Always admitting zero makes "track less / brake now" feasible;
            // it never permits a position jump or an instant reversal.
            lower_velocity = std::min(lower_velocity, 0.0);
            upper_velocity = std::max(upper_velocity, 0.0);
            frame_displacement_lower_[dof] =
                lower_velocity * frame_temporal_timestep_;
            frame_displacement_upper_[dof] =
                upper_velocity * frame_temporal_timestep_;
        }
        frame_temporal_limits_active_ = true;
    }

    void appendFrameTemporalVelocityBounds(
        double integration_timestep,
        std::vector<double>& lower,
        std::vector<double>& upper) const {
        if (!frame_temporal_limits_active_) return;
        Eigen::VectorXd displacement(nv_);
        mj_differentiatePos(
            model_, displacement.data(), 1.0,
            frame_temporal_start_qpos_.data(), data_->qpos);
        for (int dof = 0; dof < nv_; ++dof) {
            if (!std::isfinite(frame_displacement_lower_[dof]) ||
                !std::isfinite(frame_displacement_upper_[dof]))
                continue;
            lower[dof] = std::max(
                lower[dof],
                (frame_displacement_lower_[dof] - displacement[dof]) /
                    integration_timestep);
            upper[dof] = std::min(
                upper[dof],
                (frame_displacement_upper_[dof] - displacement[dof]) /
                    integration_timestep);
        }
    }

    void finishFrameTemporalLimits() {
        Eigen::VectorXd displacement(nv_);
        mj_differentiatePos(
            model_, displacement.data(), 1.0,
            frame_temporal_start_qpos_.data(), data_->qpos);
        for (int dof = 0; dof < nv_; ++dof) {
            if (!std::isfinite(frame_displacement_lower_[dof]) ||
                !std::isfinite(frame_displacement_upper_[dof]))
                continue;
            const double lower_overrun =
                frame_displacement_lower_[dof] - displacement[dof];
            const double upper_overrun =
                displacement[dof] - frame_displacement_upper_[dof];
            const double overrun = std::max({0.0, lower_overrun, upper_overrun});
            frame_temporal_diagnostics_.maximum_bound_overrun = std::max(
                frame_temporal_diagnostics_.maximum_bound_overrun, overrun);
            const double margin = std::min(
                displacement[dof] - frame_displacement_lower_[dof],
                frame_displacement_upper_[dof] - displacement[dof]);
            if (margin <= 1e-7) {
                ++frame_temporal_diagnostics_.active_bound_dofs;
                ++frame_temporal_diagnostics_.total_active_bound_dofs;
            }
        }
        previous_frame_velocity_ =
            displacement / frame_temporal_timestep_;
        frame_temporal_limits_active_ = false;
    }

    BodyMap scaleHumanData(const BodyMap& src) const {
        auto rit = src.find(human_root_name_);
        if (rit == src.end()) return src;
        const Eigen::Vector3d root_pos = rit->second.position;
        double rs = human_scale_table_.count(human_root_name_)
                    ? human_scale_table_.at(human_root_name_) : 1.0;
        Eigen::Vector3d scaled_root = root_pos * rs;
        BodyMap out;
        for (auto& [name, bd] : src) {
            if (!human_scale_table_.count(name)) continue;
            double s = human_scale_table_.at(name);
            if (name == human_root_name_)
                out[name] = {scaled_root, bd.rot_wxyz};
            else
                out[name] = {(bd.position-root_pos)*s + scaled_root, bd.rot_wxyz};
        }
        return out;
    }

    BodyMap offsetHumanData(const BodyMap& src,
                            const std::map<std::string,Eigen::Vector3d>& pos_off,
                            const std::map<std::string,Eigen::Vector4d>& rot_off) const {
        BodyMap out;
        for (auto& [name, bd] : src) {
            Eigen::Vector3d pos = bd.position;
            Eigen::Vector4d rot = bd.rot_wxyz;
            if (rot_off.count(name))
                rot = quat::normalise(quat::multiply(rot, rot_off.at(name)));
            if (pos_off.count(name))
                pos += quat::rotate(rot, pos_off.at(name));
            out[name] = {pos, rot};
        }
        return out;
    }

    BodyMap applyGroundOffset(BodyMap src) const {
        for (auto& [name,bd] : src) bd.position[2] -= ground_offset_;
        return src;
    }

    BodyMap offsetToGround(BodyMap src) const {
        double lowest = std::numeric_limits<double>::infinity();
        for (auto& [name,bd] : src) {
            std::string lo = name;
            std::transform(lo.begin(),lo.end(),lo.begin(),::tolower);
            if (lo.find("foot") != std::string::npos)
                lowest = std::min(lowest, bd.position[2]);
        }
        if (!std::isinf(lowest))
            for (auto& [name,bd] : src)
                bd.position[2] =
                    bd.position[2] - lowest + ground_clearance_;
        return src;
    }

    void clampJoints() {
        for (auto& b : vel_bounds_)
            data_->qpos[b.qadr] = std::clamp(data_->qpos[b.qadr], b.lo, b.hi);
    }

    void resetTemporalState() {
        temporal_state_initialized_ = false;
        previous_temporal_qpos_.resize(0);
        previous_previous_temporal_qpos_.resize(0);
        previous_reference_timestep_ = reference_timestep_;
    }

    void resetTemporalStateFromCurrent() {
        previous_temporal_qpos_.resize(nq_);
        previous_previous_temporal_qpos_.resize(nq_);
        for (int index = 0; index < nq_; ++index) {
            previous_temporal_qpos_[index] = data_->qpos[index];
            previous_previous_temporal_qpos_[index] = data_->qpos[index];
        }
        previous_reference_timestep_ = reference_timestep_;
        temporal_state_initialized_ = true;
    }

    void applyJointTemporalRegularization() {
        if (!motion_preserving_enabled_) return;
        if (!temporal_state_initialized_) {
            resetTemporalStateFromCurrent();
            return;
        }
        const double velocity_weight = motion_config_.joint_velocity_weight *
            temporal_regularization_weight_scale_;
        const double acceleration_weight =
            motion_config_.joint_acceleration_weight *
            temporal_regularization_weight_scale_;
        if (velocity_weight <= 0.0 && acceleration_weight <= 0.0) {
            previous_previous_temporal_qpos_ = previous_temporal_qpos_;
            for (int index = 0; index < nq_; ++index)
                previous_temporal_qpos_[index] = data_->qpos[index];
            previous_reference_timestep_ = reference_timestep_;
            return;
        }

        const double timestep_ratio = std::clamp(
            reference_timestep_ /
                std::max(1e-6, previous_reference_timestep_),
            0.25, 4.0);
        const double denominator =
            1.0 + velocity_weight + acceleration_weight;
        for (const auto& bound : vel_bounds_) {
            const double previous = previous_temporal_qpos_[bound.qadr];
            const double predicted = previous + timestep_ratio *
                (previous - previous_previous_temporal_qpos_[bound.qadr]);
            const double regularized =
                (data_->qpos[bound.qadr] + velocity_weight * previous +
                 acceleration_weight * predicted) /
                denominator;
            data_->qpos[bound.qadr] = std::clamp(
                regularized, bound.lo, bound.hi);
        }
        mj_forward(model_, data_);

        previous_previous_temporal_qpos_ = previous_temporal_qpos_;
        for (int index = 0; index < nq_; ++index)
            previous_temporal_qpos_[index] = data_->qpos[index];
        previous_reference_timestep_ = reference_timestep_;
    }

    bool contactConstraintsActive() const {
        return foot_contact_enabled_ && foot_contacts_initialized_;
    }

    int freeRootQposAddress() const {
        for (int joint = 0; joint < model_->njnt; ++joint)
            if (model_->jnt_type[joint] == mjJNT_FREE)
                return model_->jnt_qposadr[joint];
        throw std::runtime_error("contact projection requires a free root joint");
    }

    double lowestGuardHeight(
        const gmr::FootSoleDefinition& definition,
        const std::vector<Eigen::Vector3d>& guard_points) const {
        const int body = mj_name2id(model_, mjOBJ_BODY, definition.body_name.c_str());
        const Eigen::Vector3d body_position(
            data_->xpos[3 * body], data_->xpos[3 * body + 1],
            data_->xpos[3 * body + 2]);
        const Eigen::Matrix3d rotation = gmr::bodyRotation(data_, body);
        double lowest = std::numeric_limits<double>::infinity();
        for (const auto& point : guard_points)
            lowest = std::min(lowest, (body_position + rotation * point).z());
        return lowest;
    }

    int selectPrimarySupportSide() {
        auto usable = [](const FootRuntime& runtime) {
            return runtime.requested && runtime.blend > 0.0;
        };
        auto detected = [&](int side) {
            const FootRuntime& runtime = side == 0 ? left_contact_ : right_contact_;
            return usable(runtime) && !runtime.forced;
        };
        auto requested = [&](int side) {
            return usable(side == 0 ? left_contact_ : right_contact_);
        };

        if (primary_support_side_ >= 0) {
            const int other = 1 - primary_support_side_;
            if (detected(primary_support_side_) ||
                (!detected(other) && requested(primary_support_side_)))
                return primary_support_side_;
        }
        if (detected(0) != detected(1))
            primary_support_side_ = detected(0) ? 0 : 1;
        else if (detected(0) && detected(1))
            primary_support_side_ = 0;
        else if (requested(0) != requested(1))
            primary_support_side_ = requested(0) ? 0 : 1;
        else if (requested(0) && requested(1))
            primary_support_side_ = primary_support_side_ >= 0
                ? primary_support_side_ : 0;
        else
            primary_support_side_ = -1;
        return primary_support_side_;
    }

    void projectContactConfiguration(bool seat_support_foot = false) {
        const int root = freeRootQposAddress();
        const double left_lowest =
            lowestGuardHeight(foot_config_.left, left_guard_points_);
        const double right_lowest =
            lowestGuardHeight(foot_config_.right, right_guard_points_);

        double selected_height = std::min(left_lowest, right_lowest);
        bool exact_seating = seat_support_foot;
        last_root_z_safety_override_ = false;
        if (seat_support_foot && foot_contacts_initialized_) {
            const int side = selectPrimarySupportSide();
            if (side >= 0)
                selected_height = side == 0 ? left_lowest : right_lowest;
            else
                exact_seating = false;
        }
        double correction = foot_config_.ground_z - selected_height;
        if (exact_seating) {
            // Seat the selected support FK, but never lower the body far enough
            // to drive the swing sole through the floor.  The global minimum is
            // a one-sided penetration guard here, not the root-height target.
            const double global_after_support =
                std::min(left_lowest, right_lowest) + correction;
            if (global_after_support < foot_config_.ground_z)
            {
                correction += foot_config_.ground_z - global_after_support;
                last_root_z_safety_override_ = true;
            }
        }
        if (selected_height < foot_config_.ground_z ||
            (exact_seating && std::abs(correction) > 1e-12)) {
            // The gait-aware path seats exactly one persistent support sole.
            // Swing-foot geometry participates only in penetration safety, so
            // a left/right minimum switch cannot inject a root-Z discontinuity.
            data_->qpos[root + 2] += correction;
            mj_forward(model_, data_);
        }
    }

    void captureFootAnchor(FootRuntime& runtime,
                           const gmr::FootSoleDefinition& definition,
                           gmr::FootSide side) {
        const auto observation = gmr::observeFoot(model_, data_, side, definition);
        runtime.anchor_xy = observation.center_world.head<2>();
        runtime.entry_center_height =
            observation.center_world.z() - foot_config_.ground_z;
        runtime.entry_max_height = -std::numeric_limits<double>::infinity();
        for (const auto& point : observation.points_world)
            runtime.entry_max_height = std::max(
                runtime.entry_max_height,
                point.z() - foot_config_.ground_z);
        const double heel_height = 0.5 *
            (observation.points_world[0].z() +
             observation.points_world[1].z());
        const double toe_height = 0.5 *
            (observation.points_world[2].z() +
             observation.points_world[3].z());
        const double negative_y_height = 0.5 *
            (observation.points_world[0].z() +
             observation.points_world[2].z());
        const double positive_y_height = 0.5 *
            (observation.points_world[1].z() +
             observation.points_world[3].z());
        runtime.entry_heel_toe_height_difference =
            toe_height - heel_height;
        runtime.entry_lateral_height_difference =
            positive_y_height - negative_y_height;
        runtime.anchor_valid = true;
    }

    void initializeFootRuntime(FootRuntime& runtime, bool requested, bool forced,
                               const gmr::FootSoleDefinition& definition,
                               gmr::FootSide side) {
        runtime = {};
        runtime.requested = requested;
        runtime.forced = forced;
        runtime.blend = requested ? 1.0 : 0.0;
        if (requested) captureFootAnchor(runtime, definition, side);
    }

    void updateFootRuntime(FootRuntime& runtime, bool requested, bool forced,
                           const gmr::FootSoleDefinition& definition,
                           gmr::FootSide side, bool advance_transition) {
        const double step = 1.0 / static_cast<double>(
            std::max(1, foot_config_.settings.transition_frames));
        const bool entering = requested && !runtime.requested;
        const bool forced_to_detected = requested && runtime.requested &&
                                        runtime.forced && !forced;
        if ((entering || forced_to_detected) && foot_contacts_initialized_)
            captureFootAnchor(runtime, definition, side);
        runtime.requested = requested;
        runtime.forced = forced;
        if (requested) {
            if (entering) {
                runtime.blend = step;
                runtime.transition_pending_solve = true;
            } else if (advance_transition &&
                       runtime.transition_pending_solve) {
                // Contact entry is captured after following the current human
                // frame.  The next input must solve the first transition band
                // once before the band is tightened further.
                runtime.transition_pending_solve = false;
            } else if (advance_transition && last_contact_daqp_exitflag_ > 0) {
                runtime.blend = std::min(1.0, runtime.blend + step);
            }
            if (foot_contacts_initialized_ && !runtime.anchor_valid)
                captureFootAnchor(runtime, definition, side);
        } else {
            // Liftoff must release the stance constraints immediately.  A
            // multi-frame decay here makes a valid swing foot feel glued to
            // the floor and distorts turns.
            runtime.blend = 0.0;
            runtime.anchor_valid = false;
            runtime.transition_pending_solve = false;
        }
    }

    Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> pointJacobian(
        const Eigen::Vector3d& world_point, int body, bool rotational) const {
        Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> jacobian(3, nv_);
        Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor> unused(3, nv_);
        if (rotational)
            mj_jac(model_, data_, unused.data(), jacobian.data(),
                   world_point.data(), body);
        else
            mj_jac(model_, data_, jacobian.data(), unused.data(),
                   world_point.data(), body);
        return jacobian;
    }

    void addWeightedTask(Eigen::MatrixXd& H, Eigen::VectorXd& f,
                         const Eigen::MatrixXd& J,
                         const Eigen::VectorXd& target, double weight) const {
        if (weight <= 0.0 || J.rows() == 0) return;
        const Eigen::MatrixXd weighted_jacobian = weight * J;
        const Eigen::VectorXd weighted_target = weight * target;
        H += weighted_jacobian.transpose() * weighted_jacobian;
        f -= weighted_jacobian.transpose() * weighted_target;
    }

    static Eigen::Vector2d clampVectorComponents(Eigen::Vector2d value,
                                                  double maximum) {
        for (int index = 0; index < 2; ++index)
            value[index] = std::clamp(value[index], -maximum, maximum);
        return value;
    }

    Eigen::Vector2d footAnchorVelocity(
        const FootRuntime& runtime,
        const gmr::FootObservation& observation) const {
        Eigen::Vector2d error =
            runtime.anchor_xy - observation.center_world.head<2>();
        const double magnitude = error.norm();
        if (magnitude <= foot_config_.settings.anchor_deadzone)
            return Eigen::Vector2d::Zero();
        error *= (magnitude - foot_config_.settings.anchor_deadzone) / magnitude;
        Eigen::Vector2d target = foot_config_.settings.xy_kp * error;
        return clampVectorComponents(
            target, foot_config_.settings.max_anchor_correction_speed);
    }

    void addFootObjective(Eigen::MatrixXd& H, Eigen::VectorXd& f,
                          const FootRuntime& runtime,
                          const gmr::FootSoleDefinition& definition,
                          gmr::FootSide side) const {
        if (!runtime.requested || runtime.blend <= 0.0) return;
        const int body = mj_name2id(model_, mjOBJ_BODY, definition.body_name.c_str());
        const auto observation = gmr::observeFoot(model_, data_, side, definition);
        const double blend_scale = std::sqrt(runtime.blend);
        const double grounding_scale = runtime.forced
            ? foot_config_.settings.forced_support_weight_scale : 1.0;

        // A detector-forced support is only a no-flight grounding hint.  It
        // must remain free in XY so generated dance/turn motion is preserved.
        if (!runtime.forced && runtime.anchor_valid) {
            const auto center_jacobian = pointJacobian(
                observation.center_world, body, false);
            addWeightedTask(
                H, f, center_jacobian.topRows(2),
                footAnchorVelocity(runtime, observation),
                foot_config_.settings.xy_weight * blend_scale *
                    foot_contact_weight_scale_);
        }

        const auto angular_jacobian = pointJacobian(
            observation.center_world, body, true);
        const Eigen::Vector3d tilt_error =
            observation.normal_world.cross(Eigen::Vector3d::UnitZ());
        const Eigen::Vector2d angular_target =
            foot_config_.settings.tilt_kp * tilt_error.head<2>();
        addWeightedTask(
            H, f, angular_jacobian.topRows(2), angular_target,
            foot_config_.settings.tilt_weight * blend_scale * grounding_scale *
                foot_contact_weight_scale_);

        // One center-height task plus the normal task is sufficient to define
        // a flat sole.  Four independent corner tasks overconstrain the ankle
        // and send their conflict into the pelvis and waist.
        const auto center_height_jacobian = pointJacobian(
            observation.center_world, body, false);
        Eigen::MatrixXd height_row = center_height_jacobian.row(2);
        Eigen::VectorXd height_target(1);
        height_target[0] = foot_config_.settings.height_kp *
            (foot_config_.ground_z + foot_config_.settings.support_height -
             observation.center_world.z());
        height_target[0] = std::clamp(height_target[0], -0.50, 0.50);
        addWeightedTask(
            H, f, height_row, height_target,
            foot_config_.settings.height_weight * blend_scale * grounding_scale *
                foot_contact_weight_scale_);
    }

    void appendFootConstraints(
        std::vector<GeneralConstraint>& constraints,
        const FootRuntime& runtime,
        const gmr::FootSoleDefinition& definition,
        const std::vector<Eigen::Vector3d>& guard_points,
        gmr::FootSide side, double dt) const {
        const int body = mj_name2id(model_, mjOBJ_BODY, definition.body_name.c_str());
        const auto observation = gmr::observeFoot(model_, data_, side, definition);
        const Eigen::Vector3d body_position(
            data_->xpos[3 * body], data_->xpos[3 * body + 1],
            data_->xpos[3 * body + 2]);
        const Eigen::Matrix3d body_rotation = gmr::bodyRotation(data_, body);
        std::vector<Eigen::Vector3d> world_guards;
        world_guards.reserve(guard_points.size());
        for (const auto& local : guard_points)
            world_guards.push_back(body_position + body_rotation * local);
        const size_t guard_count = std::min<size_t>(8, world_guards.size());
        std::partial_sort(
            world_guards.begin(), world_guards.begin() + guard_count,
            world_guards.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.z() < rhs.z(); });
        for (size_t index = 0; index < guard_count; ++index) {
            const auto& point = world_guards[index];
            const auto jacobian = pointJacobian(point, body, false);
            GeneralConstraint floor;
            floor.row = jacobian.row(2);
            floor.lower = (foot_config_.ground_z +
                           foot_config_.settings.mesh_floor_margin -
                           foot_config_.settings.penetration_tolerance -
                           point.z()) / dt;
            floor.upper = 1e30;
            floor.relaxable = false;
            constraints.push_back(std::move(floor));
        }

        // The eight mesh vertices that are lowest before an IK step are not
        // necessarily the lowest vertices after a bounded but still sizable
        // ankle rotation.  Protect all four configured sole corners as a
        // swept-foot lookahead envelope.  A small positive margin covers the
        // linearized-Jacobian error and the 1--2 mm difference between the
        // configured corner patch and the absolute mesh minimum.
        for (const auto& point : observation.points_world) {
            const auto jacobian = pointJacobian(point, body, false);
            GeneralConstraint corner_floor;
            corner_floor.row = jacobian.row(2);
            corner_floor.lower =
                (foot_config_.ground_z +
                     foot_config_.settings.sole_corner_floor_margin -
                     foot_config_.settings.penetration_tolerance -
                     point.z()) /
                dt;
            corner_floor.upper = 1e30;
            corner_floor.relaxable = false;
            constraints.push_back(std::move(corner_floor));
        }

        // Physical non-penetration is always hard.  The remaining stance-only
        // constraints are opt-in because they trade some source-pose fidelity
        // for a physically usable support sole.
        if (!foot_config_.settings.hard_support_constraints) return;

        for (const auto& point : observation.points_world) {
            const auto jacobian = pointJacobian(point, body, false);
            if (runtime.requested && !runtime.forced &&
                runtime.anchor_valid && runtime.blend > 0.0) {
                GeneralConstraint lift;
                lift.row = jacobian.row(2);
                const double transition_clearance = (1.0 - runtime.blend) *
                    std::max(0.0, runtime.entry_max_height -
                                    foot_config_.settings.support_height_upper);
                lift.lower = -1e30;
                lift.upper = (foot_config_.ground_z +
                              foot_config_.settings.support_height_upper +
                              transition_clearance - point.z()) / dt;
                lift.relaxable = true;
                constraints.push_back(std::move(lift));
            }
        }

        if (!runtime.requested || !runtime.anchor_valid || runtime.blend <= 0.0)
            return;
        const auto center_jacobian = pointJacobian(
            observation.center_world, body, false);

        // Keep the configured sole centre inside the 1--3 mm support band.
        // This complements the mesh-wide non-penetration guard above: the
        // latter prevents tunnelling, while this band prevents a stance foot
        // from hovering or balancing on a strongly tilted edge.
        GeneralConstraint center_height;
        center_height.row = center_jacobian.row(2);
        const double entry_weight = 1.0 - runtime.blend;
        const double lower_height = foot_config_.settings.support_height +
            entry_weight * (runtime.entry_center_height -
                            foot_config_.settings.support_height);
        const double upper_height = foot_config_.settings.support_height_upper +
            entry_weight * (runtime.entry_center_height -
                            foot_config_.settings.support_height_upper);
        center_height.lower =
            (foot_config_.ground_z + lower_height -
             observation.center_world.z()) / dt;
        center_height.upper =
            (foot_config_.ground_z + upper_height -
             observation.center_world.z()) / dt;
        center_height.relaxable = true;
        constraints.push_back(std::move(center_height));

        // A horizontal ground plane only determines roll and pitch.  Constrain
        // the heel/toe and lateral height differences, but intentionally add no
        // yaw row so the source dance heading remains free.
        const Eigen::Vector3d heel = 0.5 *
            (observation.points_world[0] + observation.points_world[1]);
        const Eigen::Vector3d toe = 0.5 *
            (observation.points_world[2] + observation.points_world[3]);
        const Eigen::Vector3d negative_y = 0.5 *
            (observation.points_world[0] + observation.points_world[2]);
        const Eigen::Vector3d positive_y = 0.5 *
            (observation.points_world[1] + observation.points_world[3]);
        auto append_height_difference = [&constraints, body, dt, this](
                const Eigen::Vector3d& positive,
                const Eigen::Vector3d& negative, double tolerance) {
            const auto positive_jacobian = pointJacobian(positive, body, false);
            const auto negative_jacobian = pointJacobian(negative, body, false);
            const double difference = positive.z() - negative.z();
            GeneralConstraint flatness;
            flatness.row = positive_jacobian.row(2) -
                           negative_jacobian.row(2);
            // A newly landed dance foot can still be tilted by 5--10 cm from
            // heel to toe.  Forcing one sixth of that error away in a single
            // 30 Hz frame can conflict with the shared leg/root acceleration
            // budget.  The old infeasible fallback then held the entire robot
            // until contact was released.  Make the hard band monotonic: while
            // the sole is outside the target tolerance it may improve or stop,
            // but may never tilt farther.  The existing soft normal objective
            // spends the available temporal budget on flattening; once inside
            // 2--3 mm, this row becomes the requested strict plane bound.
            constexpr double kLinearizationEpsilon = 1e-6;
            const double feasible_tolerance = std::max(
                tolerance, std::abs(difference) + kLinearizationEpsilon);
            flatness.lower = (-feasible_tolerance - difference) / dt;
            flatness.upper = (feasible_tolerance - difference) / dt;
            // If a nonlinear Jacobian or simultaneous double support still
            // makes the plane row infeasible, retry with penetration as the
            // only hard physical constraint.  This preserves smooth bounded
            // motion instead of freezing every DoF; the plane remains a soft
            // objective and is re-enforced on the following IK step.
            flatness.relaxable = true;
            constraints.push_back(std::move(flatness));
        };
        const auto transition_tolerance = [blend = runtime.blend](
                double target, double entry_difference) {
            return target + (1.0 - blend) *
                std::max(0.0, std::abs(entry_difference) - target);
        };
        append_height_difference(
            toe, heel,
            transition_tolerance(
                foot_config_.settings.max_heel_toe_height_difference,
                runtime.entry_heel_toe_height_difference));
        append_height_difference(
            positive_y, negative_y,
            transition_tolerance(
                foot_config_.settings.max_lateral_height_difference,
                runtime.entry_lateral_height_difference));

        // Detector-forced support is deliberately free in XY, but it is still
        // the selected lower foot during an ambiguous moving double-support
        // phase.  Keep its sole flat/on-ground, then skip only the world anchor
        // and slip constraints so turning and intentional sliding remain free.
        if (runtime.forced) return;

        const Eigen::Vector2d reference = footAnchorVelocity(runtime, observation);
        // The anchor is captured at the current target pose on contact entry,
        // so XY can be constrained immediately without introducing a position
        // jump.  Only the soft pose/height weights are blended.
        const double tolerance = foot_config_.settings.max_slip_speed;
        for (int axis = 0; axis < 2; ++axis) {
            GeneralConstraint slip;
            slip.row = center_jacobian.row(axis);
            slip.lower = reference[axis] - tolerance;
            slip.upper = reference[axis] + tolerance;
            slip.relaxable = true;
            constraints.push_back(std::move(slip));
        }
    }

    std::vector<GeneralConstraint> buildFootConstraints(double dt) const {
        std::vector<GeneralConstraint> constraints;
        if (!contactConstraintsActive()) return constraints;
        appendFootConstraints(constraints, left_contact_, foot_config_.left,
                              left_guard_points_, gmr::FootSide::Left, dt);
        appendFootConstraints(constraints, right_contact_, foot_config_.right,
                              right_guard_points_, gmr::FootSide::Right, dt);
        return constraints;
    }

    void updateFootDiagnostics() {
        if (!contactConstraintsActive()) {
            foot_diagnostics_ = {};
            return;
        }
        foot_diagnostics_.enabled = true;
        foot_diagnostics_.left_stance = left_contact_.requested;
        foot_diagnostics_.right_stance = right_contact_.requested;
        foot_diagnostics_.left_forced = left_contact_.forced;
        foot_diagnostics_.right_forced = right_contact_.forced;
        foot_diagnostics_.left_blend = left_contact_.blend;
        foot_diagnostics_.right_blend = right_contact_.blend;
        foot_diagnostics_.daqp_exitflag = last_contact_daqp_exitflag_;
        foot_diagnostics_.relaxed_slip_constraints = last_contact_relaxed_;
        foot_diagnostics_.primary_support_side = primary_support_side_;
        foot_diagnostics_.root_z_safety_override =
            last_root_z_safety_override_;
        auto update = [&](const FootRuntime& runtime,
                          const gmr::FootSoleDefinition& definition,
                          gmr::FootSide side, double& slip, double& tilt,
                          double& minimum, double& maximum, double& center,
                          double& heel_toe, double& lateral) {
            const auto observation = gmr::observeFoot(model_, data_, side, definition);
            slip = runtime.anchor_valid && !runtime.forced
                ? (observation.center_world.head<2>() - runtime.anchor_xy).norm() : 0.0;
            tilt = std::acos(std::clamp(
                observation.normal_world.dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0)) *
                180.0 / std::acos(-1.0);
            minimum = std::numeric_limits<double>::infinity();
            maximum = -std::numeric_limits<double>::infinity();
            for (const auto& point : observation.points_world) {
                const double height = point.z() - foot_config_.ground_z;
                minimum = std::min(minimum, height);
                maximum = std::max(maximum, height);
            }
            center = observation.center_world.z() - foot_config_.ground_z;
            const double heel_height = 0.5 *
                (observation.points_world[0].z() +
                 observation.points_world[1].z());
            const double toe_height = 0.5 *
                (observation.points_world[2].z() +
                 observation.points_world[3].z());
            const double negative_y_height = 0.5 *
                (observation.points_world[0].z() +
                 observation.points_world[2].z());
            const double positive_y_height = 0.5 *
                (observation.points_world[1].z() +
                 observation.points_world[3].z());
            heel_toe = toe_height - heel_height;
            lateral = positive_y_height - negative_y_height;
        };
        update(left_contact_, foot_config_.left, gmr::FootSide::Left,
               foot_diagnostics_.left_slip,
               foot_diagnostics_.left_tilt_degrees,
               foot_diagnostics_.left_min_height,
               foot_diagnostics_.left_max_height,
               foot_diagnostics_.left_center_height,
               foot_diagnostics_.left_heel_toe_height_difference,
               foot_diagnostics_.left_lateral_height_difference);
        update(right_contact_, foot_config_.right, gmr::FootSide::Right,
               foot_diagnostics_.right_slip,
               foot_diagnostics_.right_tilt_degrees,
               foot_diagnostics_.right_min_height,
               foot_diagnostics_.right_max_height,
               foot_diagnostics_.right_center_height,
               foot_diagnostics_.right_heel_toe_height_difference,
               foot_diagnostics_.right_lateral_height_difference);
    }

    static double headingYaw(const Eigen::Vector4d& quaternion) {
        const Eigen::Vector4d q = quat::normalise(quaternion);
        return std::atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
    }

    static Eigen::Matrix3d quaternionRotation(
        const Eigen::Vector4d& quaternion) {
        const Eigen::Vector4d q = quat::normalise(quaternion);
        return Eigen::Quaterniond(q[0], q[1], q[2], q[3])
            .toRotationMatrix();
    }

    static double forwardPitch(const Eigen::Vector4d& quaternion) {
        const Eigen::Vector3d forward =
            quaternionRotation(quaternion).col(0);
        return std::atan2(
            -forward.z(), std::hypot(forward.x(), forward.y()));
    }

    static double wrapAngle(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void addWorldAxisTasks(
        Eigen::MatrixXd& H, Eigen::VectorXd& f,
        const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>& jacobian,
        const Eigen::Vector3d& error, const Eigen::Vector3d& axes,
        double weight) const {
        if (weight == 0.0) return;
        for (int axis = 0; axis < 3; ++axis) {
            if (axes[axis] <= 0.0) continue;
            Eigen::MatrixXd row = jacobian.row(axis);
            Eigen::VectorXd target(1);
            target[0] = error[axis];
            addWeightedTask(H, f, row, target, weight * axes[axis]);
        }
    }

    void addWorldYawTask(
        Eigen::MatrixXd& H, Eigen::VectorXd& f,
        const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>& jacobian,
        const Eigen::Vector4d& current, const Eigen::Vector4d& target,
        double weight, double axis_weight) const {
        if (weight == 0.0 || axis_weight <= 0.0) return;
        Eigen::MatrixXd row = jacobian.row(2);
        Eigen::VectorXd yaw_error(1);
        yaw_error[0] = wrapAngle(headingYaw(target) - headingYaw(current));
        addWeightedTask(H, f, row, yaw_error, weight * axis_weight);
    }

    void addTorsoPitchTask(
        Eigen::MatrixXd& H, Eigen::VectorXd& f,
        const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>& jacobian,
        const Eigen::Vector4d& current, const Eigen::Vector4d& target,
        double weight) const {
        const double task_weight =
            weight * motion_config_.torso_pitch_weight_scale;
        if (task_weight <= 0.0) return;
        const double yaw = headingYaw(current);
        const Eigen::Vector3d lateral(
            -std::sin(yaw), std::cos(yaw), 0.0);
        Eigen::MatrixXd row = lateral.transpose() * jacobian;
        Eigen::VectorXd pitch_error(1);
        pitch_error[0] = wrapAngle(
            forwardPitch(target) - forwardPitch(current));
        addWeightedTask(H, f, row, pitch_error, task_weight);
    }

    void addRootRollUprightTask(
        Eigen::MatrixXd& H, Eigen::VectorXd& f,
        const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>& jacobian,
        const Eigen::Vector4d& current,
        const Eigen::Matrix3d& current_rotation) const {
        if (motion_config_.root_roll_upright_weight <= 0.0) return;
        const double yaw = headingYaw(current);
        const Eigen::Vector3d forward(
            std::cos(yaw), std::sin(yaw), 0.0);
        const Eigen::Vector3d tilt_error =
            current_rotation.col(2).cross(Eigen::Vector3d::UnitZ());
        Eigen::MatrixXd row = forward.transpose() * jacobian;
        Eigen::VectorXd target(1);
        target[0] = forward.dot(tilt_error);
        addWeightedTask(
            H, f, row, target, motion_config_.root_roll_upright_weight);
    }

    void runIKStep(const BodyMap& targets, const std::vector<IKEntry>& entries,
                   bool include_contact)
    {
        const bool contact_active = include_contact && contactConstraintsActive();
        const double dt = model_->opt.timestep * 10.0;
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(nv_, nv_) * damping_;
        Eigen::VectorXd f = Eigen::VectorXd::Zero(nv_);

        for (const auto& entry : entries) {
            auto tgt = targets.find(entry.human_body);
            if (tgt == targets.end()) continue;
            int bid = mj_name2id(model_, mjOBJ_BODY, entry.robot_body.c_str());
            if (bid < 0) continue;

            Eigen::Matrix<double,3,Eigen::Dynamic,Eigen::RowMajor> jp_w(3,nv_), jr_w(3,nv_);
            mj_jacBody(model_, data_, jp_w.data(), jr_w.data(), bid);

            Eigen::Matrix3d R_wf;
            const mjtNum* xm = data_->xmat + bid*9;
            for (int r=0;r<3;r++) for(int c=0;c<3;c++) R_wf(r,c)=xm[r*3+c];

            Eigen::MatrixXd Jp = R_wf.transpose() * jp_w;
            Eigen::MatrixXd Jr = R_wf.transpose() * jr_w;

            Eigen::Vector3d cp(data_->xpos[bid*3],data_->xpos[bid*3+1],data_->xpos[bid*3+2]);
            Eigen::Vector3d pe = R_wf.transpose() * (tgt->second.position - cp);
            Eigen::Vector4d cq(data_->xquat[bid*4],data_->xquat[bid*4+1],
                               data_->xquat[bid*4+2],data_->xquat[bid*4+3]);
            Eigen::Vector3d re = quat::so3_error(cq, tgt->second.rot_wxyz);

            const bool gait_root = motion_preserving_enabled_ &&
                entry.robot_body == motion_config_.root_body;
            const bool gait_waist = motion_preserving_enabled_ &&
                entry.robot_body == motion_config_.waist_body;

            if (entry.pos_weight != 0) {
                if (gait_root) {
                    addWorldAxisTasks(
                        H, f, jp_w, tgt->second.position - cp,
                        motion_config_.root_position_axes,
                        entry.pos_weight);
                } else {
                    Eigen::MatrixXd wJ = entry.pos_weight * Jp;
                    Eigen::VectorXd we = entry.pos_weight * pe;
                    H += wJ.transpose() * wJ;
                    f -= wJ.transpose() * we;
                    H += 1e-4 * Eigen::MatrixXd::Identity(nv_, nv_);
                }
            }
            if (entry.rot_weight != 0) {
                if (gait_root) {
                    addWorldYawTask(
                        H, f, jr_w, cq, tgt->second.rot_wxyz,
                        entry.rot_weight,
                        motion_config_.root_rotation_axes.z());
                    addRootRollUprightTask(H, f, jr_w, cq, R_wf);
                } else if (gait_waist) {
                    addWorldYawTask(
                        H, f, jr_w, cq, tgt->second.rot_wxyz,
                        entry.rot_weight,
                        motion_config_.waist_rotation_axes.z());
                    // BUMI3 has no waist pitch joint.  Preserve forward bend
                    // by applying the G1 torso pitch to the floating trunk;
                    // the leg position tasks then realize it through hips,
                    // knees and ankles.  Torso roll remains intentionally
                    // absent from the task.
                    addTorsoPitchTask(
                        H, f, jr_w, cq, tgt->second.rot_wxyz,
                        entry.rot_weight);
                } else {
                    Eigen::MatrixXd wJ = entry.rot_weight * Jr;
                    Eigen::VectorXd we = entry.rot_weight * re;
                    H += wJ.transpose() * wJ;
                    f -= wJ.transpose() * we;
                    H += 1e-4 * Eigen::MatrixXd::Identity(nv_, nv_);
                }
            }
        }

        if (contact_active) {
            addFootObjective(H, f, left_contact_, foot_config_.left,
                             gmr::FootSide::Left);
            addFootObjective(H, f, right_contact_, foot_config_.right,
                             gmr::FootSide::Right);
        }

        std::vector<double> vlo(nv_, -1e30), vhi(nv_, 1e30);
        for (auto& b : vel_bounds_) {
            double q = data_->qpos[b.qadr];
            vlo[b.vadr] = (b.lo - q) / dt;
            vhi[b.vadr] = (b.hi - q) / dt;
        }
        if (contact_active) {
            const auto& limits = foot_config_.settings;
            for (int index = 0; index < nv_; ++index) {
                vlo[index] = std::max(vlo[index], -limits.max_joint_velocity);
                vhi[index] = std::min(vhi[index], limits.max_joint_velocity);
            }
            for (int joint = 0; joint < model_->njnt; ++joint) {
                if (model_->jnt_type[joint] != mjJNT_FREE) continue;
                const int address = model_->jnt_dofadr[joint];
                for (int axis = 0; axis < 3; ++axis) {
                    vlo[address + axis] = -limits.max_root_linear_velocity;
                    vhi[address + axis] = limits.max_root_linear_velocity;
                }
                for (int axis = 3; axis < 6; ++axis) {
                    vlo[address + axis] = -limits.max_root_angular_velocity;
                    vhi[address + axis] = limits.max_root_angular_velocity;
                }
            }
        }
        // Intersect the ordinary joint/range bounds with the displacement
        // still available in this input frame.  Because the bound is measured
        // from the frame-start configuration, all inner IK iterations and both
        // matching tables spend one shared budget instead of receiving a fresh
        // velocity allowance on every substep.
        appendFrameTemporalVelocityBounds(dt, vlo, vhi);

        std::vector<double> H_flat(nv_*nv_), f_flat(nv_);
        for (int c=0;c<nv_;c++)
            for (int r=0;r<nv_;r++)
                H_flat[r + c*nv_] = H(r,c);
        for (int i=0;i<nv_;i++) f_flat[i] = f[i];

        const std::vector<GeneralConstraint> general =
            contact_active ? buildFootConstraints(dt)
                           : std::vector<GeneralConstraint>();
        Eigen::VectorXd v(nv_);
        auto solve_qp = [&](bool include_relaxable, Eigen::VectorXd& solution) {
            std::vector<const GeneralConstraint*> selected;
            for (const auto& constraint : general)
                if (include_relaxable || !constraint.relaxable)
                    selected.push_back(&constraint);
            const int rows = static_cast<int>(selected.size());
            const int total_constraints = nv_ + rows;
            std::vector<double> lower(total_constraints), upper(total_constraints);
            for (int index = 0; index < nv_; ++index) {
                lower[index] = vlo[index];
                upper[index] = vhi[index];
            }
            std::vector<double> A_flat(static_cast<size_t>(rows) * nv_);
            for (int row = 0; row < rows; ++row) {
                lower[nv_ + row] = selected[row]->lower;
                upper[nv_ + row] = selected[row]->upper;
                for (int column = 0; column < nv_; ++column)
                    A_flat[row * nv_ + column] = selected[row]->row[column];
            }
            std::vector<int> sense(total_constraints, 0);
            DAQPProblem qp;
            qp.n = nv_;
            qp.m = total_constraints;
            qp.ms = nv_;
            qp.H = H_flat.data();
            qp.f = f_flat.data();
            qp.A = rows > 0 ? A_flat.data() : nullptr;
            qp.bupper = upper.data();
            qp.blower = lower.data();
            qp.sense = sense.data();
            qp.nh = 0;
            qp.break_points = nullptr;

            DAQPResult result{};
            std::vector<double> x_solution(nv_, 0.0);
            std::vector<double> lambda(total_constraints, 0.0);
            result.x = x_solution.data();
            result.lam = lambda.data();
            DAQPSettings settings;
            daqp_default_settings(&settings);
            daqp_quadprog(&result, &qp, &settings);
            if (result.exitflag > 0)
                for (int index = 0; index < nv_; ++index)
                    solution[index] = result.x[index];
            return result.exitflag;
        };

        int exitflag = solve_qp(true, v);
        bool relaxed = false;
        if (exitflag <= 0 && !general.empty() &&
            std::any_of(general.begin(), general.end(),
                        [](const auto& constraint) { return constraint.relaxable; })) {
            relaxed = true;
            exitflag = solve_qp(false, v);
        }
        if (contact_active) {
            last_contact_daqp_exitflag_ = exitflag;
            last_contact_relaxed_ = relaxed;
        }

        if (exitflag > 0) {
            // solution was populated by solve_qp
        } else {
            static auto last_daqp_fail = std::chrono::steady_clock::now() - std::chrono::seconds(10);
            auto now_daqp_fail = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now_daqp_fail - last_daqp_fail).count() >= 1.0) {
                last_daqp_fail = now_daqp_fail;
                std::printf("[WARN][GMR] daqp failed exitflag=%d%s\n",
                            exitflag,
                            contact_active
                                ? "; holding configuration because contact constraints are active"
                                : "; falling back to LDLT");
            }
            if (contact_active || frame_temporal_limits_active_) {
                v.setZero();
            } else {
                v = H.ldlt().solve(-f);
                for (auto& b : vel_bounds_)
                    v[b.vadr] = std::clamp(v[b.vadr], vlo[b.vadr], vhi[b.vadr]);
            }
        }

        mj_integratePos(model_, data_->qpos, v.data(), dt);
        clampJoints();
        mj_forward(model_, data_);
        if (contact_active && !frame_temporal_limits_active_)
            projectContactConfiguration();
    }

    void solveIK(const BodyMap& targets, const std::vector<IKEntry>& entries,
                 bool include_contact) {
        double curr_error = computeError(targets, entries, include_contact);
        runIKStep(targets, entries, include_contact);
        double next_error = computeError(targets, entries, include_contact);
        int num_iter = 0;
        while (curr_error - next_error > 0.001 && num_iter < 10) {
            curr_error = next_error;
            runIKStep(targets, entries, include_contact);
            next_error = computeError(targets, entries, include_contact);
            num_iter++;
        }
    }

    double computeError(const BodyMap& targets, const std::vector<IKEntry>& entries,
                        bool include_contact) const {
        double sq = 0.0;
        for (const auto& entry : entries) {
            auto it = targets.find(entry.human_body);
            if (it == targets.end()) continue;
            int bid = mj_name2id(model_, mjOBJ_BODY, entry.robot_body.c_str());
            if (bid < 0) continue;
            Eigen::Vector3d cp(data_->xpos[bid*3],data_->xpos[bid*3+1],data_->xpos[bid*3+2]);
            Eigen::Vector4d cq(data_->xquat[bid*4],data_->xquat[bid*4+1],
                            data_->xquat[bid*4+2],data_->xquat[bid*4+3]);
            const bool gait_root = motion_preserving_enabled_ &&
                entry.robot_body == motion_config_.root_body;
            const bool gait_waist = motion_preserving_enabled_ &&
                entry.robot_body == motion_config_.waist_body;
            double pos_err = (it->second.position - cp).squaredNorm();
            double rot_err = quat::so3_error(
                cq, it->second.rot_wxyz).squaredNorm();
            if (gait_root) {
                const Eigen::Vector3d difference =
                    (it->second.position - cp).cwiseProduct(
                        motion_config_.root_position_axes);
                pos_err = difference.squaredNorm();
                const double yaw_error = wrapAngle(
                    headingYaw(it->second.rot_wxyz) - headingYaw(cq));
                rot_err = motion_config_.root_rotation_axes.z() *
                    yaw_error * yaw_error;
                const mjtNum* matrix = data_->xmat + 9 * bid;
                const Eigen::Vector3d body_z(
                    matrix[2], matrix[5], matrix[8]);
                const double yaw = headingYaw(cq);
                const Eigen::Vector3d forward(
                    std::cos(yaw), std::sin(yaw), 0.0);
                const Eigen::Vector3d upright_error =
                    body_z.cross(Eigen::Vector3d::UnitZ());
                const double roll_error = forward.dot(upright_error);
                rot_err += roll_error * roll_error;
            } else if (gait_waist) {
                const double yaw_error = wrapAngle(
                    headingYaw(it->second.rot_wxyz) - headingYaw(cq));
                rot_err = motion_config_.waist_rotation_axes.z() *
                    yaw_error * yaw_error;
                const double pitch_error = wrapAngle(
                    forwardPitch(it->second.rot_wxyz) - forwardPitch(cq));
                rot_err += motion_config_.torso_pitch_weight_scale *
                    pitch_error * pitch_error;
            }
            sq += pos_err + rot_err;
        }
        if (include_contact && contactConstraintsActive() &&
            foot_contact_weight_scale_ > 0.0) {
            auto add_contact_error = [&](
                    const FootRuntime& runtime,
                    const gmr::FootSoleDefinition& definition,
                    gmr::FootSide side) {
                if (!runtime.anchor_valid || runtime.blend <= 0.0) return;
                const auto observation =
                    gmr::observeFoot(model_, data_, side, definition);
                if (!runtime.forced)
                    sq += runtime.blend *
                        (observation.center_world.head<2>() - runtime.anchor_xy)
                            .squaredNorm();
                const Eigen::Vector3d tilt =
                    observation.normal_world.cross(Eigen::Vector3d::UnitZ());
                sq += runtime.blend * tilt.head<2>().squaredNorm();
                const double height_error =
                    observation.center_world.z() - foot_config_.ground_z -
                    foot_config_.settings.support_height;
                sq += runtime.blend * height_error * height_error;
            };
            add_contact_error(left_contact_, foot_config_.left,
                              gmr::FootSide::Left);
            add_contact_error(right_contact_, foot_config_.right,
                              gmr::FootSide::Right);
        }
        return std::sqrt(sq);
    }
};

} // namespace gmr_mink
