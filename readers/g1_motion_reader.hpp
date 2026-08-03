#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

namespace gmr {

struct G1MotionFrame {
    Eigen::Vector3d root_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond root_rotation = Eigen::Quaterniond::Identity();
    std::vector<double> joint_positions;
    double timestamp = 0.0;
};

class G1MotionReader {
public:
    explicit G1MotionReader(double fps = 30.0);

    bool load_npy(const std::string& path);
    bool load_json_frame(const std::string& json);
    bool get_next_frame(G1MotionFrame& frame);

    void set_fps(double fps);
    double fps() const { return fps_; }
    size_t frame_count() const { return frames_.size(); }
    void rewind() { cursor_ = 0; }
    const std::string& last_error() const { return last_error_; }

private:
    bool validate(G1MotionFrame& frame, const std::string& source);

    double fps_ = 30.0;
    std::vector<G1MotionFrame> frames_;
    std::deque<G1MotionFrame> json_frames_;
    size_t cursor_ = 0;
    std::string last_error_;
};

}  // namespace gmr
