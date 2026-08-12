#include "adapters/g1_motion_adapter.hpp"
#include "gmr/foot_contact_json.hpp"
#include "gmr/geometry_ground.hpp"
#include "gmr/gmr_mink.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

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

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

double rootForwardPitch(const Eigen::VectorXd& qpos) {
    const Eigen::Quaterniond rotation(
        qpos[3], qpos[4], qpos[5], qpos[6]);
    const Eigen::Vector3d forward = rotation.normalized() *
        Eigen::Vector3d::UnitX();
    return std::atan2(
        -forward.z(), std::hypot(forward.x(), forward.y()));
}

gmr::G1MotionFrame neutralFrame() {
    gmr::G1MotionFrame frame;
    frame.root_position = Eigen::Vector3d(0.0, 0.0, 0.793);
    frame.root_rotation = Eigen::Quaterniond::Identity();
    frame.joint_positions.assign(29, 0.0);
    return frame;
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
        const nlohmann::json config = readJson(config_path);
        const auto target_feet = config.at("root").at("target_foot_bodies")
            .get<std::vector<std::string>>();

        gmr::G1MotionAdapter adapter(source_xml);
        gmr::G1MotionFrame frame = neutralFrame();
        const gmr::BodyMap neutral = adapter.to_body_map(frame);
        gmr::FootContactState double_support;
        double_support.left_stance = true;
        double_support.right_stance = true;

        gmr_mink::GMR constrained(target_xml, config_path, 1.0, 1.0, false);
        require(constrained.footContactConfigured(),
                "G1 to BUMI3 config must enable foot contacts");
        require(constrained.motionPreservingConfigured(),
                "G1 to BUMI3 config must define the gait-aware profile");
        constrained.setMotionPreservingEnabled(true);
        constrained.setReferenceTimestep(1.0 / 50.0);
        Eigen::VectorXd qpos;
        for (int iteration = 0; iteration < 100; ++iteration)
            qpos = constrained.retarget(neutral, false);
        gmr::TargetGroundAligner ground(target_xml, target_feet);
        ground.align(qpos, 0.0);
        constrained.setConfiguration(qpos);
        constrained.initializeFootContacts(double_support);

        // Deliberately ask the source root and ankles to drag/tilt both stance
        // feet.  Contact-aware IK must absorb this conflict in the free body
        // and remaining joints instead of moving the sole anchors.
        frame.root_position.x() += 0.10;
        const auto& joints = gmr::G1MotionAdapter::omg_joint_order();
        for (size_t index = 0; index < joints.size(); ++index) {
            if (joints[index] == "left_ankle_pitch_joint" ||
                joints[index] == "right_ankle_pitch_joint")
                frame.joint_positions[index] = 0.30;
        }
        const gmr::BodyMap disturbed = adapter.to_body_map(frame);
        for (int frame_index = 0; frame_index < 60; ++frame_index) {
            constrained.setFootContactState(double_support);
            qpos = constrained.retarget(disturbed, false);
        }
        const auto diagnostics = constrained.footContactDiagnostics();
        std::cout << "[light-contact] slip=(" << diagnostics.left_slip << ','
                  << diagnostics.right_slip << ")m tilt=("
                  << diagnostics.left_tilt_degrees << ','
                  << diagnostics.right_tilt_degrees << ")deg height_max=("
                  << diagnostics.left_max_height << ','
                  << diagnostics.right_max_height << ")m\n";
        require(qpos.allFinite(), "constrained output contains non-finite values");
        require(diagnostics.daqp_exitflag > 0,
                "contact DAQP did not return a feasible solution");
        require(!diagnostics.relaxed_slip_constraints,
                "contact DAQP unexpectedly relaxed stance constraints");
        require(diagnostics.left_slip < 0.09 && diagnostics.right_slip < 0.09,
                "soft anchor did not oppose a 100 mm disturbance at all");
        require(diagnostics.left_tilt_degrees < 10.0 &&
                    diagnostics.right_tilt_degrees < 10.0,
                "light contact allowed an extreme stance-foot tilt");
        require(diagnostics.left_max_height < 0.025 &&
                    diagnostics.right_max_height < 0.025,
                "light contact allowed an extreme stance-foot lift");
        require(diagnostics.left_min_height >= -0.001 &&
                    diagnostics.right_min_height >= -0.001,
                "stance sole penetrates below tolerance");
        require(diagnostics.primary_support_side == 0,
                "double support did not select a persistent primary foot");

        // Liftoff is not blended as a hard constraint: the old anchor must be
        // gone on the first swing frame so turns and steps do not feel glued.
        gmr::FootContactState right_support;
        right_support.right_stance = true;
        constrained.setFootContactState(right_support);
        qpos = constrained.retarget(disturbed, false);
        const auto released = constrained.footContactDiagnostics();
        require(!released.left_stance && released.left_blend == 0.0 &&
                    released.left_slip == 0.0,
                "swing foot retained a stale stance anchor");
        require(released.primary_support_side == 1,
                "root-Z policy did not switch to the detected right support");

        // Merely adding the optional config block must not change callers that
        // never initialize contact state.  Explicitly disabling the feature
        // must produce the exact same unconstrained IK trajectory.
        gmr_mink::GMR passive(target_xml, config_path, 1.0, 1.0, false);
        gmr_mink::GMR legacy(target_xml, config_path, 1.0, 1.0, false);
        legacy.setFootContactEnabled(false);
        Eigen::VectorXd passive_qpos;
        Eigen::VectorXd legacy_qpos;
        for (int iteration = 0; iteration < 100; ++iteration) {
            passive_qpos = passive.retarget(neutral, false);
            legacy_qpos = legacy.retarget(neutral, false);
            require((passive_qpos - legacy_qpos).cwiseAbs().maxCoeff() < 1e-12,
                    "optional contact hook changed the legacy IK path");
        }

        // Explicitly disabled mode also retains the legacy post-align path.
        ground.align(legacy_qpos, 0.0);
        require(std::abs(ground.lowest(legacy_qpos)) < 1e-9,
                "legacy ground-align compatibility failed");

        // BUMI3 only has waist yaw.  A pure G1 torso roll target must therefore
        // be ignored by the gait-aware profile instead of being pushed into the
        // BUMI3 floating pelvis.  The default/legacy solver must remain
        // responsive to the original full-orientation task.
        gmr_mink::GMR yaw_only(target_xml, config_path, 1.0, 1.0, false);
        gmr_mink::GMR full_orientation(
            target_xml, config_path, 1.0, 1.0, false);
        yaw_only.setFootContactEnabled(false);
        yaw_only.setMotionPreservingEnabled(true);
        yaw_only.setReferenceTimestep(1.0 / 50.0);
        full_orientation.setFootContactEnabled(false);
        Eigen::VectorXd yaw_neutral;
        Eigen::VectorXd full_neutral;
        for (int iteration = 0; iteration < 100; ++iteration) {
            yaw_neutral = yaw_only.retarget(neutral, false);
            full_neutral = full_orientation.retarget(neutral, false);
        }
        gmr::BodyMap torso_roll = neutral;
        const Eigen::Quaterniond original_torso(
            torso_roll.at("torso_link").rot_wxyz[0],
            torso_roll.at("torso_link").rot_wxyz[1],
            torso_roll.at("torso_link").rot_wxyz[2],
            torso_roll.at("torso_link").rot_wxyz[3]);
        const double original_yaw = std::atan2(
            2.0 * (original_torso.w() * original_torso.z() +
                   original_torso.x() * original_torso.y()),
            1.0 - 2.0 * (original_torso.y() * original_torso.y() +
                         original_torso.z() * original_torso.z()));
        const Eigen::Quaterniond rolled =
            Eigen::AngleAxisd(original_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitX());
        torso_roll.at("torso_link").rot_wxyz = Eigen::Vector4d(
            rolled.w(), rolled.x(), rolled.y(), rolled.z());
        Eigen::VectorXd yaw_rolled;
        Eigen::VectorXd full_rolled;
        for (int iteration = 0; iteration < 30; ++iteration) {
            yaw_rolled = yaw_only.retarget(torso_roll, false);
            full_rolled = full_orientation.retarget(torso_roll, false);
        }
        const double yaw_only_change =
            (yaw_rolled - yaw_neutral).cwiseAbs().maxCoeff();
        const double full_orientation_change =
            (full_rolled - full_neutral).cwiseAbs().maxCoeff();
        std::cout << "[waist-axis] yaw_only_change=" << yaw_only_change
                  << " legacy_change=" << full_orientation_change << '\n';
        require(yaw_only_change < 0.005,
                "yaw-only profile reacted to discarded G1 torso roll");
        require(full_orientation_change > 0.01 &&
                    full_orientation_change > 10.0 * yaw_only_change,
                "legacy full-orientation task no longer reacts to torso roll");

        // Discarding torso roll must not discard forward bending.  BUMI3 has
        // no waist-pitch joint, so the gait-aware solver maps G1 torso pitch
        // onto the floating trunk and lets both hip-pitch joints compensate
        // against the unchanged leg/foot targets.
        gmr_mink::GMR pitch_aware(
            target_xml, config_path, 1.0, 1.0, false);
        pitch_aware.setFootContactEnabled(false);
        pitch_aware.setMotionPreservingEnabled(true);
        pitch_aware.setReferenceTimestep(1.0 / 50.0);
        Eigen::VectorXd pitch_neutral;
        for (int iteration = 0; iteration < 100; ++iteration)
            pitch_neutral = pitch_aware.retarget(neutral, false);
        gmr::BodyMap torso_pitch = neutral;
        const Eigen::Quaterniond pitched =
            Eigen::AngleAxisd(original_yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(0.55, Eigen::Vector3d::UnitY());
        torso_pitch.at("torso_link").rot_wxyz = Eigen::Vector4d(
            pitched.w(), pitched.x(), pitched.y(), pitched.z());
        Eigen::VectorXd pitch_bent;
        for (int iteration = 0; iteration < 40; ++iteration)
            pitch_bent = pitch_aware.retarget(torso_pitch, false);

        char model_error[1024] = {};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> target_model(
            mj_loadXML(target_xml.c_str(), nullptr, model_error,
                       sizeof(model_error)),
            &mj_deleteModel);
        require(target_model != nullptr,
                std::string("failed to load BUMI3 model: ") + model_error);
        const int left_hip_id = mj_name2id(
            target_model.get(), mjOBJ_JOINT, "l_leg_pitch_joint");
        const int right_hip_id = mj_name2id(
            target_model.get(), mjOBJ_JOINT, "r_leg_pitch_joint");
        require(left_hip_id >= 0 && right_hip_id >= 0,
                "BUMI3 hip-pitch joints are missing");
        const int left_hip_qpos = target_model->jnt_qposadr[left_hip_id];
        const int right_hip_qpos = target_model->jnt_qposadr[right_hip_id];
        const double trunk_pitch_change =
            rootForwardPitch(pitch_bent) - rootForwardPitch(pitch_neutral);
        const double hip_pitch_change = std::max(
            std::abs(pitch_bent[left_hip_qpos] -
                     pitch_neutral[left_hip_qpos]),
            std::abs(pitch_bent[right_hip_qpos] -
                     pitch_neutral[right_hip_qpos]));
        std::cout << "[torso-pitch] trunk_change=" << trunk_pitch_change
                  << " hip_change=" << hip_pitch_change << '\n';
        require(trunk_pitch_change > 0.15,
                "gait-aware profile discarded G1 forward torso pitch");
        require(hip_pitch_change > 0.03,
                "BUMI3 hips did not compensate the forward trunk bend");

        // The temporal terms are opt-in with the gait profile and must soften
        // a one-frame joint target step without changing the legacy solver.
        gmr::G1MotionFrame stepped_frame = neutralFrame();
        for (size_t index = 0; index < joints.size(); ++index)
            if (joints[index] == "left_hip_pitch_joint" ||
                joints[index] == "right_hip_pitch_joint")
                stepped_frame.joint_positions[index] = 0.45;
        const gmr::BodyMap stepped = adapter.to_body_map(stepped_frame);
        gmr_mink::GMR temporally_regularized(
            target_xml, config_path, 1.0, 1.0, false);
        gmr_mink::GMR temporal_control(
            target_xml, config_path, 1.0, 1.0, false);
        for (auto* solver : {&temporally_regularized, &temporal_control}) {
            solver->setFootContactEnabled(false);
            solver->setMotionPreservingEnabled(true);
            solver->setReferenceTimestep(1.0 / 50.0);
        }
        temporal_control.setTemporalRegularizationWeightScale(0.0);
        Eigen::VectorXd regularized_neutral;
        Eigen::VectorXd control_neutral;
        for (int iteration = 0; iteration < 100; ++iteration) {
            regularized_neutral = temporally_regularized.retarget(neutral, false);
            control_neutral = temporal_control.retarget(neutral, false);
        }
        const Eigen::VectorXd regularized_step =
            temporally_regularized.retarget(stepped, false);
        const Eigen::VectorXd control_step =
            temporal_control.retarget(stepped, false);
        const double regularized_joint_step =
            (regularized_step.tail(regularized_step.size() - 7) -
             regularized_neutral.tail(regularized_neutral.size() - 7))
                .cwiseAbs().maxCoeff();
        const double control_joint_step =
            (control_step.tail(control_step.size() - 7) -
             control_neutral.tail(control_neutral.size() - 7))
                .cwiseAbs().maxCoeff();
        std::cout << "[temporal] regularized_step=" << regularized_joint_step
                  << " raw_step=" << control_joint_step << '\n';
        require(regularized_joint_step < control_joint_step,
                "joint velocity/acceleration terms did not soften a target step");

        std::cout << "g1_bumi3_foot_constraint_test: PASS slip=("
                  << diagnostics.left_slip << ',' << diagnostics.right_slip
                  << ")m tilt=(" << diagnostics.left_tilt_degrees << ','
                  << diagnostics.right_tilt_degrees << ")deg\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "g1_bumi3_foot_constraint_test: FAIL: "
                  << error.what() << '\n';
        return 1;
    }
}
