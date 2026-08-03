#pragma once

#include "gmr/body_map.hpp"
#include "readers/g1_motion_reader.hpp"

#include <mujoco/mujoco.h>

#include <memory>
#include <string>
#include <vector>

namespace gmr {

class G1MotionAdapter {
public:
    explicit G1MotionAdapter(const std::string& g1_xml_path);

    BodyMap to_body_map(const G1MotionFrame& frame);
    const Eigen::VectorXd& source_qpos() const { return source_qpos_; }
    const mjModel* model() const { return model_.get(); }
    const mjData* data() const { return data_.get(); }

    static const std::vector<std::string>& omg_joint_order();
    static const std::vector<std::string>& source_body_names();

private:
    struct ModelDeleter { void operator()(mjModel* value) const { mj_deleteModel(value); } };
    struct DataDeleter { void operator()(mjData* value) const { mj_deleteData(value); } };

    std::unique_ptr<mjModel, ModelDeleter> model_;
    std::unique_ptr<mjData, DataDeleter> data_;
    std::vector<int> joint_qpos_addresses_;
    std::vector<int> body_ids_;
    int root_qpos_address_ = -1;
    Eigen::VectorXd source_qpos_;
};

}  // namespace gmr
