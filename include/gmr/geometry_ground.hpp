#pragma once

#include <Eigen/Dense>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace gmr {

inline Eigen::Vector3d transformPoint(const mjtNum* position,
                                      const mjtNum* rotation,
                                      const Eigen::Vector3d& local) {
    Eigen::Vector3d world(position[0], position[1], position[2]);
    for (int row = 0; row < 3; ++row) {
        world[row] += rotation[3 * row] * local[0] +
                      rotation[3 * row + 1] * local[1] +
                      rotation[3 * row + 2] * local[2];
    }
    return world;
}

inline double lowestGeometryZ(const mjModel* model, const mjData* data,
                              const std::vector<std::string>& body_names,
                              bool contact_only = true) {
    if (!model || !data) throw std::runtime_error("MuJoCo model/data is null");
    double lowest = std::numeric_limits<double>::infinity();
    int considered = 0;
    for (const std::string& body_name : body_names) {
        const int body = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
        if (body < 0) throw std::runtime_error("missing foot body: " + body_name);
        for (int geom = 0; geom < model->ngeom; ++geom) {
            if (model->geom_bodyid[geom] != body) continue;
            if (contact_only && model->geom_contype[geom] == 0 &&
                model->geom_conaffinity[geom] == 0) continue;
            ++considered;
            const mjtNum* position = data->geom_xpos + 3 * geom;
            const mjtNum* rotation = data->geom_xmat + 9 * geom;
            if (model->geom_type[geom] == mjGEOM_MESH &&
                model->geom_dataid[geom] >= 0) {
                const int mesh = model->geom_dataid[geom];
                const int begin = model->mesh_vertadr[mesh];
                const int count = model->mesh_vertnum[mesh];
                for (int index = 0; index < count; ++index) {
                    const float* vertex = model->mesh_vert + 3 * (begin + index);
                    const Eigen::Vector3d local(vertex[0], vertex[1], vertex[2]);
                    lowest = std::min(lowest,
                                      transformPoint(position, rotation, local).z());
                }
            } else {
                const mjtNum* bounds = model->geom_aabb + 6 * geom;
                for (int x : {-1, 1}) for (int y : {-1, 1}) for (int z : {-1, 1}) {
                    const Eigen::Vector3d local(
                        bounds[0] + x * bounds[3],
                        bounds[1] + y * bounds[4],
                        bounds[2] + z * bounds[5]);
                    lowest = std::min(lowest,
                                      transformPoint(position, rotation, local).z());
                }
            }
        }
    }
    if (considered == 0 || !std::isfinite(lowest))
        throw std::runtime_error("no usable foot contact geometry found");
    return lowest;
}

class TargetGroundAligner {
public:
    TargetGroundAligner(const std::string& xml_path,
                        std::vector<std::string> foot_bodies)
        : foot_bodies_(std::move(foot_bodies)) {
        char error[1024] = {};
        model_.reset(mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error)));
        if (!model_) throw std::runtime_error("TargetGroundAligner: " +
                                              std::string(error));
        data_.reset(mj_makeData(model_.get()));
        if (!data_) throw std::runtime_error("TargetGroundAligner: mj_makeData failed");
        for (const std::string& body : foot_bodies_) {
            if (mj_name2id(model_.get(), mjOBJ_BODY, body.c_str()) < 0)
                throw std::runtime_error("TargetGroundAligner missing body: " + body);
        }
    }

    double align(Eigen::VectorXd& qpos, double height_offset = 0.0) {
        if (!std::isfinite(height_offset))
            throw std::runtime_error("height_offset must be finite");
        if (qpos.size() != model_->nq)
            throw std::runtime_error("ground align qpos size mismatch");
        for (int index = 0; index < model_->nq; ++index)
            data_->qpos[index] = qpos[index];
        mj_forward(model_.get(), data_.get());
        const double before = lowestGeometryZ(
            model_.get(), data_.get(), foot_bodies_, true);
        const int root = freeRootQposAddress();
        qpos[root + 2] += height_offset - before;
        return before;
    }

    double lowest(const Eigen::VectorXd& qpos) {
        if (qpos.size() != model_->nq)
            throw std::runtime_error("ground query qpos size mismatch");
        for (int index = 0; index < model_->nq; ++index)
            data_->qpos[index] = qpos[index];
        mj_forward(model_.get(), data_.get());
        return lowestGeometryZ(model_.get(), data_.get(), foot_bodies_, true);
    }

    const mjModel* model() const { return model_.get(); }

private:
    int freeRootQposAddress() const {
        for (int joint = 0; joint < model_->njnt; ++joint) {
            if (model_->jnt_type[joint] == mjJNT_FREE)
                return model_->jnt_qposadr[joint];
        }
        throw std::runtime_error("target model has no free root joint");
    }

    struct ModelDeleter { void operator()(mjModel* value) const { mj_deleteModel(value); } };
    struct DataDeleter { void operator()(mjData* value) const { mj_deleteData(value); } };
    std::unique_ptr<mjModel, ModelDeleter> model_;
    std::unique_ptr<mjData, DataDeleter> data_;
    std::vector<std::string> foot_bodies_;
};

}  // namespace gmr
