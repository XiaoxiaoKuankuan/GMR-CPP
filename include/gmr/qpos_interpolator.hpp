#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gmr {

// Causal one-source-frame interpolation for viewer/GMT references.  When a
// new retargeted pose arrives, playback moves from the currently displayed
// pose to that target over the measured source interval.  This adds roughly
// one source frame of latency while removing sample-and-hold stepping.
class QposInterpolator {
public:
    struct Config {
        double default_interval_seconds = 0.02;
        double minimum_interval_seconds = 0.005;
        double maximum_interval_seconds = 0.10;
    };

    QposInterpolator() : QposInterpolator(Config()) {}

    explicit QposInterpolator(Config config) : config_(config) {
        if (!std::isfinite(config_.default_interval_seconds) ||
            !std::isfinite(config_.minimum_interval_seconds) ||
            !std::isfinite(config_.maximum_interval_seconds) ||
            config_.minimum_interval_seconds <= 0.0 ||
            config_.maximum_interval_seconds <
                config_.minimum_interval_seconds ||
            config_.default_interval_seconds <
                config_.minimum_interval_seconds ||
            config_.default_interval_seconds >
                config_.maximum_interval_seconds)
            throw std::runtime_error("invalid qpos interpolation intervals");
    }

    bool initialized() const { return initialized_; }

    void reset(const Eigen::VectorXd& qpos, double source_timestamp,
               double local_time) {
        validateQpos(qpos);
        if (!std::isfinite(local_time))
            throw std::runtime_error("qpos interpolation local time is not finite");
        from_ = qpos;
        target_ = qpos;
        start_local_time_ = local_time;
        duration_ = config_.default_interval_seconds;
        last_source_timestamp_ = source_timestamp;
        initialized_ = true;
    }

    void push(const Eigen::VectorXd& qpos, double source_timestamp,
              double local_time) {
        validateQpos(qpos);
        if (!std::isfinite(local_time))
            throw std::runtime_error("qpos interpolation local time is not finite");
        if (!initialized_) {
            reset(qpos, source_timestamp, local_time);
            return;
        }
        if (qpos.size() != target_.size())
            throw std::runtime_error("qpos interpolation size changed");

        const Eigen::VectorXd current = sample(local_time);
        double interval = config_.default_interval_seconds;
        if (std::isfinite(source_timestamp) &&
            std::isfinite(last_source_timestamp_) &&
            source_timestamp > last_source_timestamp_) {
            interval = std::clamp(
                source_timestamp - last_source_timestamp_,
                config_.minimum_interval_seconds,
                config_.maximum_interval_seconds);
        }
        from_ = current;
        target_ = qpos;
        start_local_time_ = local_time;
        duration_ = interval;
        last_source_timestamp_ = source_timestamp;
    }

    Eigen::VectorXd sample(double local_time) const {
        if (!initialized_)
            throw std::runtime_error("qpos interpolator has no reference");
        if (!std::isfinite(local_time))
            throw std::runtime_error("qpos interpolation local time is not finite");
        const double phase = std::clamp(
            (local_time - start_local_time_) / duration_, 0.0, 1.0);
        Eigen::VectorXd result = (1.0 - phase) * from_ + phase * target_;

        Eigen::Quaterniond from_quaternion(
            from_[3], from_[4], from_[5], from_[6]);
        Eigen::Quaterniond target_quaternion(
            target_[3], target_[4], target_[5], target_[6]);
        from_quaternion.normalize();
        target_quaternion.normalize();
        if (from_quaternion.dot(target_quaternion) < 0.0)
            target_quaternion.coeffs() *= -1.0;
        const Eigen::Quaterniond interpolated =
            from_quaternion.slerp(phase, target_quaternion).normalized();
        result[3] = interpolated.w();
        result[4] = interpolated.x();
        result[5] = interpolated.y();
        result[6] = interpolated.z();
        return result;
    }

private:
    static void validateQpos(const Eigen::VectorXd& qpos) {
        if (qpos.size() < 7 || !qpos.allFinite())
            throw std::runtime_error(
                "qpos interpolation requires at least seven finite values");
        const double quaternion_norm = qpos.segment<4>(3).norm();
        if (quaternion_norm < 1e-9)
            throw std::runtime_error("qpos interpolation quaternion is zero");
    }

    Config config_;
    bool initialized_ = false;
    Eigen::VectorXd from_;
    Eigen::VectorXd target_;
    double start_local_time_ = 0.0;
    double duration_ = 0.02;
    double last_source_timestamp_ = 0.0;
};

}  // namespace gmr
