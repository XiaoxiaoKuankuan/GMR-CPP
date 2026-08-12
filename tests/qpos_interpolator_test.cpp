#include "gmr/qpos_interpolator.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

Eigen::VectorXd pose(double x, double yaw, double joint) {
    Eigen::VectorXd value = Eigen::VectorXd::Zero(8);
    value[0] = x;
    const Eigen::Quaterniond rotation(
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    value[3] = rotation.w();
    value[4] = rotation.x();
    value[5] = rotation.y();
    value[6] = rotation.z();
    value[7] = joint;
    return value;
}

}  // namespace

int main() {
    try {
        gmr::QposInterpolator::Config config;
        config.default_interval_seconds = 0.02;
        gmr::QposInterpolator interpolator(config);
        const double pi = std::acos(-1.0);
        interpolator.reset(pose(0.0, 0.0, 0.0), 10.0, 20.0);
        interpolator.push(pose(1.0, pi / 2.0, 2.0), 10.02, 20.02);

        const Eigen::VectorXd middle = interpolator.sample(20.03);
        require(std::abs(middle[0] - 0.5) < 1e-9,
                "root translation is not linearly interpolated");
        require(std::abs(middle[7] - 1.0) < 1e-9,
                "joint position is not linearly interpolated");
        const Eigen::Quaterniond middle_rotation(
            middle[3], middle[4], middle[5], middle[6]);
        const Eigen::Vector3d direction =
            middle_rotation * Eigen::Vector3d::UnitX();
        require(std::abs(direction.x() - std::sqrt(0.5)) < 1e-9 &&
                    std::abs(direction.y() - std::sqrt(0.5)) < 1e-9,
                "root quaternion did not use shortest-path slerp");

        const Eigen::VectorXd end = interpolator.sample(20.04);
        require((end - pose(1.0, pi / 2.0, 2.0)).norm() < 1e-9,
                "interpolation did not reach the target");

        interpolator.push(pose(2.0, pi, 4.0), 10.04, 20.04);
        require((interpolator.sample(20.04) - end).norm() < 1e-9,
                "pushing a new target introduced a discontinuity");

        std::cout << "qpos_interpolator_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "qpos_interpolator_test: FAIL: " << error.what() << '\n';
        return 1;
    }
}
