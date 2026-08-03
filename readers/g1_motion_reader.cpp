#include "g1_motion_reader.hpp"

#include "gmr/npy_io.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <stdexcept>

namespace gmr {

namespace {
constexpr size_t kQposDim = 36;
constexpr size_t kJointCount = 29;
}

G1MotionReader::G1MotionReader(double fps) {
    set_fps(fps);
}

void G1MotionReader::set_fps(double fps) {
    if (!std::isfinite(fps) || fps <= 0.0)
        throw std::runtime_error("G1 motion fps must be positive and finite");
    fps_ = fps;
}

bool G1MotionReader::validate(G1MotionFrame& frame, const std::string& source) {
    if (!frame.root_position.allFinite() ||
        !std::isfinite(frame.root_rotation.w()) ||
        !std::isfinite(frame.root_rotation.x()) ||
        !std::isfinite(frame.root_rotation.y()) ||
        !std::isfinite(frame.root_rotation.z()) ||
        !std::isfinite(frame.timestamp)) {
        last_error_ = source + ": non-finite root pose/timestamp";
        return false;
    }
    if (frame.joint_positions.size() != kJointCount) {
        last_error_ = source + ": expected 29 joints, got " +
                      std::to_string(frame.joint_positions.size());
        return false;
    }
    for (double value : frame.joint_positions) {
        if (!std::isfinite(value)) {
            last_error_ = source + ": non-finite joint position";
            return false;
        }
    }
    const double norm = frame.root_rotation.norm();
    if (!std::isfinite(norm) || norm < 1e-10) {
        last_error_ = source + ": root quaternion has zero norm";
        return false;
    }
    frame.root_rotation.normalize();
    return true;
}

bool G1MotionReader::load_npy(const std::string& path) {
    try {
        const npy::Array2D array = npy::load2D(path);
        if (array.cols != kQposDim)
            throw std::runtime_error("expected shape (T,36), got second dimension " +
                                     std::to_string(array.cols));
        std::vector<G1MotionFrame> loaded;
        loaded.reserve(array.rows);
        for (size_t row = 0; row < array.rows; ++row) {
            const double* q = array.values.data() + row * array.cols;
            G1MotionFrame frame;
            frame.root_position = Eigen::Vector3d(q[0], q[1], q[2]);
            frame.root_rotation = Eigen::Quaterniond(q[3], q[4], q[5], q[6]);
            frame.joint_positions.assign(q + 7, q + 36);
            frame.timestamp = static_cast<double>(row) / fps_;
            if (!validate(frame, path + " frame " + std::to_string(row)))
                return false;
            loaded.push_back(std::move(frame));
        }
        frames_ = std::move(loaded);
        cursor_ = 0;
        last_error_.clear();
        return true;
    } catch (const std::exception& error) {
        last_error_ = error.what();
        return false;
    }
}

bool G1MotionReader::load_json_frame(const std::string& json_text) {
    try {
        const nlohmann::json value = nlohmann::json::parse(json_text);
        G1MotionFrame frame;
        const auto& pos = value.at("root_pos");
        const auto& quat = value.at("root_quat");
        const auto& joints = value.at("joints");
        if (!pos.is_array() || pos.size() != 3 ||
            !quat.is_array() || quat.size() != 4 ||
            !joints.is_array() || joints.size() != kJointCount) {
            throw std::runtime_error(
                "JSON requires root_pos[3], root_quat[4] wxyz and joints[29]");
        }
        frame.root_position = Eigen::Vector3d(
            pos.at(0).get<double>(), pos.at(1).get<double>(),
            pos.at(2).get<double>());
        frame.root_rotation = Eigen::Quaterniond(
            quat.at(0).get<double>(), quat.at(1).get<double>(),
            quat.at(2).get<double>(), quat.at(3).get<double>());
        frame.joint_positions = joints.get<std::vector<double>>();
        frame.timestamp = value.at("timestamp").get<double>();
        if (!validate(frame, "G1 JSON frame")) return false;
        json_frames_.push_back(std::move(frame));
        last_error_.clear();
        return true;
    } catch (const std::exception& error) {
        last_error_ = error.what();
        return false;
    }
}

bool G1MotionReader::get_next_frame(G1MotionFrame& frame) {
    if (!json_frames_.empty()) {
        frame = std::move(json_frames_.front());
        json_frames_.pop_front();
        return true;
    }
    if (cursor_ >= frames_.size()) return false;
    frame = frames_[cursor_++];
    return true;
}

}  // namespace gmr
