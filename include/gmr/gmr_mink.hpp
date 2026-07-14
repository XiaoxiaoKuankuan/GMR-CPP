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
    }

    ~GMR() {
        if (data_)  mj_deleteData(data_);
        if (model_) mj_deleteModel(model_);
    }
    GMR(const GMR&) = delete;
    GMR& operator=(const GMR&) = delete;

    Eigen::VectorXd retarget(const BodyMap& human_data, bool offset_to_ground = false)
    {
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
        if (use_table1_) solveIK(offset, entries1_);
        if (use_table2_) solveIK(offset, entries2_);

        Eigen::VectorXd out(nq_);
        for (int i = 0; i < nq_; ++i) out[i] = data_->qpos[i];
        return out;
    }

    const BodyMap& getScaledHumanData() const { return scaled_human_data_; }
    void setGroundOffset(double g) { ground_offset_ = g; }

private:
    mjModel* model_ = nullptr;
    mjData*  data_  = nullptr;
    int nv_, nq_;
    double damping_;
    bool   verbose_;

    std::string human_root_name_;
    double ground_height_ = 0.0;
    double ground_offset_ = 0.0;
    bool use_table1_ = false, use_table2_ = false;

    std::map<std::string,double>          human_scale_table_;
    std::map<std::string,Eigen::Vector3d> pos_offsets1_, pos_offsets2_;
    std::map<std::string,Eigen::Vector4d> rot_offsets1_, rot_offsets2_;
    std::vector<IKEntry> entries1_, entries2_;
    BodyMap scaled_human_data_;

    int frame_count_ = 0;

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
                bd.position[2] = bd.position[2] - lowest + 0.06;
        return src;
    }

    void clampJoints() {
        for (auto& b : vel_bounds_)
            data_->qpos[b.qadr] = std::clamp(data_->qpos[b.qadr], b.lo, b.hi);
    }

    void runIKStep(const BodyMap& targets, const std::vector<IKEntry>& entries)
    {
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

            if (entry.pos_weight != 0) {
                Eigen::MatrixXd wJ = entry.pos_weight * Jp;
                Eigen::VectorXd we = entry.pos_weight * pe;
                H += wJ.transpose() * wJ;
                f -= wJ.transpose() * we;
                H += 1e-4 * Eigen::MatrixXd::Identity(nv_, nv_);
            }
            if (entry.rot_weight != 0) {
                Eigen::MatrixXd wJ = entry.rot_weight * Jr;
                Eigen::VectorXd we = entry.rot_weight * re;
                H += wJ.transpose() * wJ;
                f -= wJ.transpose() * we;
                H += 1e-4 * Eigen::MatrixXd::Identity(nv_, nv_);
            }
        }

        std::vector<double> vlo(nv_, -1e30), vhi(nv_, 1e30);
        for (auto& b : vel_bounds_) {
            double q = data_->qpos[b.qadr];
            vlo[b.vadr] = (b.lo - q) / dt;
            vhi[b.vadr] = (b.hi - q) / dt;
        }

        std::vector<double> H_flat(nv_*nv_), f_flat(nv_);
        for (int c=0;c<nv_;c++)
            for (int r=0;r<nv_;r++)
                H_flat[r + c*nv_] = H(r,c);
        for (int i=0;i<nv_;i++) f_flat[i] = f[i];

        DAQPProblem qp;
        qp.n=nv_; qp.m=nv_; qp.ms=nv_;
        qp.H=H_flat.data(); qp.f=f_flat.data();
        qp.A=nullptr; qp.bupper=vhi.data(); qp.blower=vlo.data();
        qp.sense=nullptr; qp.nh=0; qp.break_points=nullptr;

        DAQPResult res;
        std::vector<double> x_sol(nv_,0.0), lam_sol(nv_,0.0);
        res.x=x_sol.data(); res.lam=lam_sol.data();

        DAQPSettings settings;
        daqp_default_settings(&settings);
        daqp_quadprog(&res, &qp, &settings);

        Eigen::VectorXd v(nv_);
        if (res.exitflag > 0) {
            for (int i=0;i<nv_;i++) v[i]=res.x[i];
        } else {
            static auto last_daqp_fail = std::chrono::steady_clock::now() - std::chrono::seconds(10);
            auto now_daqp_fail = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now_daqp_fail - last_daqp_fail).count() >= 1.0) {
                last_daqp_fail = now_daqp_fail;
                std::printf("[WARN][GMR] daqp failed exitflag=%d, falling back to LDLT\n",
                            res.exitflag);
            }
            v = H.ldlt().solve(-f);
            for (auto& b : vel_bounds_)
                v[b.vadr] = std::clamp(v[b.vadr], vlo[b.vadr], vhi[b.vadr]);
        }

        mj_integratePos(model_, data_->qpos, v.data(), dt);
        clampJoints();
        mj_forward(model_, data_);
    }

    void solveIK(const BodyMap& targets, const std::vector<IKEntry>& entries) {
        double curr_error = computeError(targets, entries);
        runIKStep(targets, entries);
        double next_error = computeError(targets, entries);
        int num_iter = 0;
        while (curr_error - next_error > 0.001 && num_iter < 10) {
            curr_error = next_error;
            runIKStep(targets, entries);
            next_error = computeError(targets, entries);
            num_iter++;
        }
    }

    double computeError(const BodyMap& targets, const std::vector<IKEntry>& entries) const {
        double sq = 0.0;
        for (const auto& entry : entries) {
            auto it = targets.find(entry.human_body);
            if (it == targets.end()) continue;
            int bid = mj_name2id(model_, mjOBJ_BODY, entry.robot_body.c_str());
            if (bid < 0) continue;
            Eigen::Vector3d cp(data_->xpos[bid*3],data_->xpos[bid*3+1],data_->xpos[bid*3+2]);
            Eigen::Vector4d cq(data_->xquat[bid*4],data_->xquat[bid*4+1],
                            data_->xquat[bid*4+2],data_->xquat[bid*4+3]);
            double pos_err = (it->second.position - cp).squaredNorm();
            double rot_err = quat::so3_error(cq, it->second.rot_wxyz).squaredNorm();
            sq += pos_err + rot_err;
        }
        return std::sqrt(sq);
    }
};

} // namespace gmr_mink