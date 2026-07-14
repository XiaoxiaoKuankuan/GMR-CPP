#pragma once
/**
 * redis_publisher.hpp — Redis frame publisher (multi-robot)
 *
 * Velocity filtering: per-axis EMA on lin_vel and ang_vel.
 * qpos is NOT filtered — keeps phase response of pose intact.
 * Trade-off: (q, v) are not strictly congruent. Acceptable when v is used
 * as a soft reference; for strict congruence, filter q and re-differentiate.
 */

#include "body_map.hpp"
#include <hiredis/hiredis.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cstring>
#include <cstdio>
#include <stdexcept>
#include <cmath>
#include <chrono>
#include <iostream>

namespace gmr {

// ── Quaternion helpers ───────────────────────────────────────────────────────
struct QuatWXYZ { double w, x, y, z; };

inline QuatWXYZ normaliseQ(QuatWXYZ q) {
    double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if (n < 1e-12) return {1,0,0,0};
    return {q.w/n, q.x/n, q.y/n, q.z/n};
}
inline QuatWXYZ conjugateQ(QuatWXYZ q) { return {q.w,-q.x,-q.y,-q.z}; }
inline QuatWXYZ mulQ(QuatWXYZ a, QuatWXYZ b) {
    return { a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
             a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
             a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
             a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w };
}
inline Eigen::Vector3d so3AngVel(QuatWXYZ prev, QuatWXYZ next, double dt) {
    if (dt <= 1e-8) return Eigen::Vector3d::Zero();
    QuatWXYZ rel = normaliseQ(mulQ(next, conjugateQ(normaliseQ(prev))));
    double w     = std::clamp(rel.w, -1.0, 1.0);
    double angle = 2.0 * std::acos(w);
    double s     = std::sqrt(std::max(1e-12, 1.0 - w*w));
    return Eigen::Vector3d(rel.x/s, rel.y/s, rel.z/s) * (angle / dt);
}

// ── G1 / E1 joint reorder maps ────────────────────────────────────────────
static constexpr int G1_NUM_JOINTS = 29;
static constexpr int G1_JOINT_IDS_MAP[G1_NUM_JOINTS] = {
     0,  6, 12,  1,  7, 13,  2,  8, 14,  3,
     9, 15, 22,  4, 10, 16, 23,  5, 11, 17,
    24, 18, 25, 19, 26, 20, 27, 21, 28
};

static constexpr int E1_NUM_JOINTS = 24;
static constexpr int E1_JOINT_IDS_MAP[E1_NUM_JOINTS] = {
     0,  6, 12,  1,  7, 13,  2,  8, 14, 19,
     3,  9, 15, 20,  4, 10, 16, 21,  5, 11,
    17, 22, 18, 23
};

struct RobotPreset {
    int              num_joints;
    std::vector<int> joint_ids_map;
    std::string      default_key;
    double           pelvis_z_offset = -0.05;
};

inline RobotPreset presetG1() {
    RobotPreset p;
    p.num_joints       = G1_NUM_JOINTS;
    p.default_key      = "mmocap_motion_frame_g1";
    p.pelvis_z_offset  = -0.05;
    p.joint_ids_map.assign(G1_JOINT_IDS_MAP,
                           G1_JOINT_IDS_MAP + G1_NUM_JOINTS);
    return p;
}

inline RobotPreset presetE1() {
    RobotPreset p;
    p.num_joints       = E1_NUM_JOINTS;
    p.default_key      = "gmt_online_frame_e1";
    p.pelvis_z_offset  = 0.0;
    p.joint_ids_map.assign(E1_JOINT_IDS_MAP,
                           E1_JOINT_IDS_MAP + E1_NUM_JOINTS);
    return p;
}

// ── RedisPublisher ───────────────────────────────────────────────────────
class RedisPublisher {
public:
    struct Config {
        std::string host            = "127.0.0.1";
        int         port            = 6379;
        int         db              = 0;
        std::string key             = "mmocap_motion_frame_g1";
        int         ttl_ms          = 200;
        double      pelvis_z_offset = -0.05;

        int              num_joints    = G1_NUM_JOINTS;
        std::vector<int> joint_ids_map;

        // ── Velocity filtering ─────────────────────────────────────────
        // EMA: v_filt = alpha * v_raw + (1-alpha) * v_filt_prev
        // Smaller alpha = stronger smoothing, more lag.
        // 1.0 disables EMA by default; pass a smaller alpha to enable smoothing.
        double lin_vel_alpha = 1.0;   // 0 = freeze, 1 = no filter
        double ang_vel_alpha = 1.0;
        // Reject single-frame spikes that exceed these magnitudes.
        // 0 disables the limit.
        double lin_vel_max   = 0.0;   // m/s
        double ang_vel_max   = 0.0;   // rad/s
        bool   publish_raw_bones = true;

        void applyPreset(const RobotPreset& p) {
            num_joints      = p.num_joints;
            joint_ids_map   = p.joint_ids_map;
            pelvis_z_offset = p.pelvis_z_offset;
            if (key == "mmocap_motion_frame_g1" || key.empty())
                key = p.default_key;
        }
    };

    explicit RedisPublisher(Config cfg) : cfg_(std::move(cfg)) {
        num_joints_   = cfg_.num_joints;
        frame_floats_ = 1 + 3 + 4 + 3 + 3 + num_joints_;
        reorder_      = !cfg_.joint_ids_map.empty();

        // Clamp filter alphas to [0, 1]
        cfg_.lin_vel_alpha = std::clamp(cfg_.lin_vel_alpha, 0.0, 1.0);
        cfg_.ang_vel_alpha = std::clamp(cfg_.ang_vel_alpha, 0.0, 1.0);
        cfg_.lin_vel_max   = std::max(0.0, cfg_.lin_vel_max);
        cfg_.ang_vel_max   = std::max(0.0, cfg_.ang_vel_max);

        if (reorder_ && (int)cfg_.joint_ids_map.size() != num_joints_)
            throw std::runtime_error(
                "[RedisPublisher] joint_ids_map size mismatch: got " +
                std::to_string(cfg_.joint_ids_map.size()) +
                " expected " + std::to_string(num_joints_));

        redis_ = redisConnect(cfg_.host.c_str(), cfg_.port);
        if (!redis_ || redis_->err)
            throw std::runtime_error("[RedisPublisher] " +
                std::string(redis_ ? redis_->errstr : "alloc failed"));
        if (cfg_.db) {
            auto* reply = (redisReply*)redisCommand(redis_, "SELECT %d", cfg_.db);
            if (reply) freeReplyObject(reply);
        }

        std::cout << "[RedisPublisher] " << cfg_.host << ":" << cfg_.port
                  << "/" << cfg_.db << " key=" << cfg_.key
                  << " stream=" << cfg_.key << ":stream"
                  << " joints=" << num_joints_
                  << " reorder=" << (reorder_ ? "yes" : "no")
                  << " frame=" << frame_floats_ << " floats"
                  << " vel_filt=(lin_a=" << cfg_.lin_vel_alpha
                  << ", ang_a=" << cfg_.ang_vel_alpha
                  << ", lin_max=" << cfg_.lin_vel_max
                  << ", ang_max=" << cfg_.ang_vel_max << ")\n";
    }

    ~RedisPublisher() {
        if (redis_) redisFree(redis_);
    }

    void clearKey() {
        auto* reply = (redisReply*)redisCommand(redis_, "DEL %s", cfg_.key.c_str());
        if (reply) freeReplyObject(reply);
        const std::string stream_key = cfg_.key + ":stream";
        reply = (redisReply*)redisCommand(redis_, "DEL %s", stream_key.c_str());
        if (reply) freeReplyObject(reply);
    }

    /// Reset velocity filter state. Call after long pauses / re-seeding.
    void resetVelocityFilter() {
        has_prev_     = false;
        vel_filt_init_ = false;
        lin_vel_filt_  = Eigen::Vector3d::Zero();
        ang_vel_filt_  = Eigen::Vector3d::Zero();
    }

    bool publish(const Eigen::VectorXd& qpos,
                 double frame_time,
                 const BodyMap& raw_bones = {})
    {
        if (qpos.size() < 7 + num_joints_) return false;
        // Publisher can tick faster than the retarget thread produces fresh
        // frames during brief IK jitter. Hold and republish the latest pose for
        // duplicate timestamps so the downstream controller still receives a
        // steady-rate command stream.
        const bool duplicate_frame =
            has_prev_ && std::abs(frame_time - prev_frame_time_) <= 1e-6;
        if (has_prev_ && frame_time < prev_frame_time_ - 1e-6) return false;

        Eigen::VectorXd q = qpos;
        if (q.size() > 2) q[2] += cfg_.pelvis_z_offset;

        Eigen::Vector3d root_pos(q[0], q[1], q[2]);
        QuatWXYZ root_quat = normaliseQ({q[3], q[4], q[5], q[6]});

        // ── Raw differential velocity ─────────────────────────────────
        Eigen::Vector3d lin_vel_raw = Eigen::Vector3d::Zero();
        Eigen::Vector3d ang_vel_raw = Eigen::Vector3d::Zero();
        bool have_raw_vel = false;
        if (has_prev_ && !duplicate_frame) {
            double dt = std::clamp(frame_time - prev_frame_time_, 0.006, 0.5);
            lin_vel_raw = (root_pos - prev_pos_) / dt;
            ang_vel_raw = so3AngVel(prev_quat_, root_quat, dt);
            have_raw_vel = true;
        }

        // ── Spike rejection (drop, don't clamp — clamping warps direction) ─
        bool spike = false;
        if (have_raw_vel) {
            if (cfg_.lin_vel_max > 0 && lin_vel_raw.norm() > cfg_.lin_vel_max) spike = true;
            if (cfg_.ang_vel_max > 0 && ang_vel_raw.norm() > cfg_.ang_vel_max) spike = true;
        }

        // ── EMA filter ────────────────────────────────────────────────
        Eigen::Vector3d lin_vel_out = Eigen::Vector3d::Zero();
        Eigen::Vector3d ang_vel_out = Eigen::Vector3d::Zero();
        if (have_raw_vel && !spike) {
            if (!vel_filt_init_) {
                lin_vel_filt_ = lin_vel_raw;
                ang_vel_filt_ = ang_vel_raw;
                vel_filt_init_ = true;
            } else {
                const double a_lin = cfg_.lin_vel_alpha;
                const double a_ang = cfg_.ang_vel_alpha;
                lin_vel_filt_ = a_lin * lin_vel_raw + (1.0 - a_lin) * lin_vel_filt_;
                ang_vel_filt_ = a_ang * ang_vel_raw + (1.0 - a_ang) * ang_vel_filt_;
            }
            lin_vel_out = lin_vel_filt_;
            ang_vel_out = ang_vel_filt_;
        } else if (vel_filt_init_) {
            // Spike or no raw vel this tick — hold the last filtered value
            // rather than emitting zero (zero would itself be a spike for downstream).
            lin_vel_out = lin_vel_filt_;
            ang_vel_out = ang_vel_filt_;
            if (spike) {
                static int spike_dbg = 0;
                if (spike_dbg++ < 20)
                    std::printf("[RedisPublisher] vel spike rejected: "
                                "|lv|=%.2f |av|=%.2f\n",
                                lin_vel_raw.norm(), ang_vel_raw.norm());
            }
        }

        // ── Update prev state ─────────────────────────────────────────
        if (!duplicate_frame) {
            prev_pos_        = root_pos;
            prev_quat_       = root_quat;
            prev_frame_time_ = frame_time;
            has_prev_        = true;
        }

        // ── Joint positions (with optional reordering) ────────────────
        std::vector<double> jp(num_joints_);
        if (reorder_) {
            for (int i = 0; i < num_joints_; ++i)
                jp[i] = q[7 + cfg_.joint_ids_map[i]];
        } else {
            for (int i = 0; i < num_joints_; ++i)
                jp[i] = q[7 + i];
        }

        double stamp = epochSec();
        auto payload = pack(stamp, root_pos, root_quat,
                            lin_vel_out, ang_vel_out, jp);
        std::vector<uint8_t> raw_payload;
        if (cfg_.publish_raw_bones && !raw_bones.empty())
            raw_payload = packRawBones(raw_bones);
        publishFrameBinary(cfg_.key, payload,
                           raw_payload.empty() ? nullptr : &raw_payload);

        frames_sent_++;
        if (frames_sent_ % 200 == 0) {
            double pub_hz = updatePubHz();
            std::printf("[RedisPublisher] pub=%.1fHz frames=%lld stamp=%.3f "
                        "|lv|=%.3f |av|=%.3f\n",
                        pub_hz, frames_sent_, stamp,
                        lin_vel_out.norm(), ang_vel_out.norm());
        }
        return true;
    }

    bool publishWithVel(const Eigen::VectorXd& qpos,
                        double t_sec,
                        const Eigen::Vector3d& lin_vel,
                        const Eigen::Vector3d& ang_vel,
                        const BodyMap& raw_bones = {})
    {
        if (qpos.size() < 7 + num_joints_) return false;

        Eigen::VectorXd q = qpos;
        if (q.size() > 2) q[2] += cfg_.pelvis_z_offset;

        Eigen::Vector3d root_pos(q[0], q[1], q[2]);
        QuatWXYZ root_quat = normaliseQ({q[3], q[4], q[5], q[6]});

        std::vector<double> jp(num_joints_);
        if (reorder_) {
            for (int i = 0; i < num_joints_; ++i)
                jp[i] = q[7 + cfg_.joint_ids_map[i]];
        } else {
            for (int i = 0; i < num_joints_; ++i)
                jp[i] = q[7 + i];
        }

        auto payload = pack(t_sec, root_pos, root_quat, lin_vel, ang_vel, jp);
        std::vector<uint8_t> raw_payload;
        if (cfg_.publish_raw_bones && !raw_bones.empty())
            raw_payload = packRawBones(raw_bones);
        publishFrameBinary(cfg_.key, payload,
                           raw_payload.empty() ? nullptr : &raw_payload);

        frames_sent_++;
        if (frames_sent_ % 200 == 0) {
            double pub_hz = updatePubHz();
            std::printf("[RedisPublisher] pub=%.1fHz frames=%lld t=%.3f\n",
                        pub_hz, frames_sent_, t_sec);
        }
        return true;
    }

private:
    double updatePubHz() {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - pub_rate_t0_).count();
        long long delta = frames_sent_ - pub_rate_frames0_;
        pub_rate_t0_ = now;
        pub_rate_frames0_ = frames_sent_;
        return elapsed > 1e-9 ? delta / elapsed : 0.0;
    }

    std::vector<uint8_t> pack(
        double stamp,
        const Eigen::Vector3d& pos, QuatWXYZ quat,
        const Eigen::Vector3d& lv, const Eigen::Vector3d& av,
        const std::vector<double>& jp)
    {
        std::vector<float> arr(frame_floats_);
        quat = normaliseQ(quat);
        arr[0]  = float(stamp);
        arr[1]  = float(pos.x());  arr[2]  = float(pos.y());  arr[3]  = float(pos.z());
        arr[4]  = float(quat.w);   arr[5]  = float(quat.x);
        arr[6]  = float(quat.y);   arr[7]  = float(quat.z);
        arr[8]  = float(lv.x());   arr[9]  = float(lv.y());   arr[10] = float(lv.z());
        arr[11] = float(av.x());   arr[12] = float(av.y());   arr[13] = float(av.z());
        for (int i = 0; i < num_joints_; ++i) arr[14+i] = float(jp[i]);
        std::vector<uint8_t> buf(arr.size() * sizeof(float));
        std::memcpy(buf.data(), arr.data(), buf.size());
        return buf;
    }

    int appendSetBinary(const std::string& key, const std::vector<uint8_t>& p) {
        if (cfg_.ttl_ms > 0)
            return redisAppendCommand(redis_, "PSETEX %s %d %b",
                key.c_str(), cfg_.ttl_ms, p.data(), p.size());
        return redisAppendCommand(redis_, "SET %s %b",
            key.c_str(), p.data(), p.size());
    }

    void drainReplies(int n) {
        for (int i = 0; i < n; ++i) {
            void* reply = nullptr;
            if (redisGetReply(redis_, &reply) != REDIS_OK) return;
            if (reply) freeReplyObject(reply);
        }
    }

    void publishBinary(const std::string& key, const std::vector<uint8_t>& p) {
        redisReply* reply = nullptr;
        if (cfg_.ttl_ms > 0)
            reply = (redisReply*)redisCommand(redis_, "PSETEX %s %d %b",
                key.c_str(), cfg_.ttl_ms, p.data(), p.size());
        else
            reply = (redisReply*)redisCommand(redis_, "SET %s %b",
                key.c_str(), p.data(), p.size());
        if (reply) freeReplyObject(reply);
    }

    void publishFrameBinary(const std::string& key, const std::vector<uint8_t>& p,
                            const std::vector<uint8_t>* raw_bones_payload = nullptr) {
        const std::string stream_key = key + ":stream";
        int queued = 0;
        if (appendSetBinary(key, p) == REDIS_OK) queued++;
        if (redisAppendCommand(redis_, "XADD %s MAXLEN ~ %d * frame %b",
                               stream_key.c_str(), 512,
                               p.data(), p.size()) == REDIS_OK) {
            queued++;
        }
        if (raw_bones_payload) {
            if (appendSetBinary(key + "_raw_bones", *raw_bones_payload) == REDIS_OK)
                queued++;
        }
        drainReplies(queued);
    }

    std::vector<uint8_t> packRawBones(const BodyMap& bones) {
        std::string json = "{";
        bool first = true;
        for (auto& [name, bd] : bones) {
            if (!first) json += ",";
            first = false;
            char buf[256];
            snprintf(buf, sizeof(buf),
                "\"%s\":[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f]",
                name.c_str(),
                bd.position[0], bd.position[1], bd.position[2],
                bd.rot_wxyz[0], bd.rot_wxyz[1], bd.rot_wxyz[2], bd.rot_wxyz[3]);
            json += buf;
        }
        json += "}";
        return std::vector<uint8_t>(json.begin(), json.end());
    }

    static double epochSec() {
        static auto t0 = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
    }

    Config        cfg_;
    redisContext* redis_ = nullptr;
    long long     frames_sent_ = 0;
    long long     pub_rate_frames0_ = 0;
    std::chrono::steady_clock::time_point pub_rate_t0_ = std::chrono::steady_clock::now();
    int           num_joints_;
    int           frame_floats_;
    bool          reorder_;

    bool            has_prev_        = false;
    double          prev_frame_time_ = 0.0;
    Eigen::Vector3d prev_pos_        = Eigen::Vector3d::Zero();
    QuatWXYZ        prev_quat_       = {1,0,0,0};

    // Velocity filter state
    bool            vel_filt_init_ = false;
    Eigen::Vector3d lin_vel_filt_  = Eigen::Vector3d::Zero();
    Eigen::Vector3d ang_vel_filt_  = Eigen::Vector3d::Zero();
};

} // namespace gmr
