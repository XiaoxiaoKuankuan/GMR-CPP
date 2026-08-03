#include "g1_motion_adapter.hpp"

#include <cmath>
#include <stdexcept>

namespace gmr {

const std::vector<std::string>& G1MotionAdapter::omg_joint_order() {
    static const std::vector<std::string> names = {
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
        "left_wrist_pitch_joint", "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
        "right_wrist_pitch_joint", "right_wrist_yaw_joint",
    };
    return names;
}

const std::vector<std::string>& G1MotionAdapter::source_body_names() {
    static const std::vector<std::string> names = {
        "pelvis", "torso_link",
        "left_hip_roll_link", "left_knee_link", "left_ankle_roll_link",
        "right_hip_roll_link", "right_knee_link", "right_ankle_roll_link",
        "left_shoulder_yaw_link", "left_elbow_link",
        "right_shoulder_yaw_link", "right_elbow_link",
    };
    return names;
}

G1MotionAdapter::G1MotionAdapter(const std::string& g1_xml_path) {
    char error[1024] = {};
    model_.reset(mj_loadXML(g1_xml_path.c_str(), nullptr, error, sizeof(error)));
    if (!model_) throw std::runtime_error("G1MotionAdapter: " + std::string(error));
    data_.reset(mj_makeData(model_.get()));
    if (!data_) throw std::runtime_error("G1MotionAdapter: mj_makeData failed");
    if (model_->nq != 36)
        throw std::runtime_error("G1 source model must have nq=36, got " +
                                 std::to_string(model_->nq));

    const int pelvis = mj_name2id(model_.get(), mjOBJ_BODY, "pelvis");
    if (pelvis < 0) throw std::runtime_error("G1 source model has no pelvis body");
    for (int joint = 0; joint < model_->njnt; ++joint) {
        if (model_->jnt_bodyid[joint] == pelvis &&
            model_->jnt_type[joint] == mjJNT_FREE) {
            root_qpos_address_ = model_->jnt_qposadr[joint];
            break;
        }
    }
    if (root_qpos_address_ < 0)
        throw std::runtime_error("G1 pelvis does not own a free joint");

    for (const std::string& name : omg_joint_order()) {
        const int joint = mj_name2id(model_.get(), mjOBJ_JOINT, name.c_str());
        if (joint < 0) throw std::runtime_error("G1 source missing joint: " + name);
        if (model_->jnt_type[joint] != mjJNT_HINGE &&
            model_->jnt_type[joint] != mjJNT_SLIDE)
            throw std::runtime_error("G1 OMG joint is not one-DoF: " + name);
        joint_qpos_addresses_.push_back(model_->jnt_qposadr[joint]);
    }
    for (const std::string& name : source_body_names()) {
        const int body = mj_name2id(model_.get(), mjOBJ_BODY, name.c_str());
        if (body < 0) throw std::runtime_error("G1 source missing body: " + name);
        body_ids_.push_back(body);
    }
    source_qpos_.resize(model_->nq);
}

BodyMap G1MotionAdapter::to_body_map(const G1MotionFrame& frame) {
    if (frame.joint_positions.size() != omg_joint_order().size())
        throw std::runtime_error("G1MotionAdapter expected 29 joint positions");
    mj_resetData(model_.get(), data_.get());
    data_->qpos[root_qpos_address_ + 0] = frame.root_position.x();
    data_->qpos[root_qpos_address_ + 1] = frame.root_position.y();
    data_->qpos[root_qpos_address_ + 2] = frame.root_position.z();
    Eigen::Quaterniond quaternion = frame.root_rotation.normalized();
    data_->qpos[root_qpos_address_ + 3] = quaternion.w();
    data_->qpos[root_qpos_address_ + 4] = quaternion.x();
    data_->qpos[root_qpos_address_ + 5] = quaternion.y();
    data_->qpos[root_qpos_address_ + 6] = quaternion.z();
    for (size_t index = 0; index < joint_qpos_addresses_.size(); ++index)
        data_->qpos[joint_qpos_addresses_[index]] = frame.joint_positions[index];
    mj_forward(model_.get(), data_.get());

    for (int index = 0; index < model_->nq; ++index)
        source_qpos_[index] = data_->qpos[index];
    BodyMap result;
    const auto& names = source_body_names();
    for (size_t index = 0; index < names.size(); ++index) {
        const int body = body_ids_[index];
        BodyData pose;
        pose.position = Eigen::Vector3d(
            data_->xpos[3 * body], data_->xpos[3 * body + 1],
            data_->xpos[3 * body + 2]);
        pose.rot_wxyz = Eigen::Vector4d(
            data_->xquat[4 * body], data_->xquat[4 * body + 1],
            data_->xquat[4 * body + 2], data_->xquat[4 * body + 3]);
        result.emplace(names[index], pose);
    }
    return result;
}

}  // namespace gmr
