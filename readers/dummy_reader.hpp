#pragma once
/**
 * dummy_reader.hpp — Fake skeleton data reader for debugging
 *
 * Generates a static T-pose at a fixed rate (default 30Hz).
 * Bone names and positions match smplx_to_g1.json canonical names.
 * No hardware needed — just include and use instead of PicoReader.
 */

#include "base_reader.hpp"
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>

namespace gmr {

class DummyReader : public BaseReader {
public:
    struct Config {
        double hz = 30.0;  // fake data rate
    };

    explicit DummyReader(FrameQueue& queue)
        : BaseReader(queue), cfg_(Config{}) {}

    DummyReader(FrameQueue& queue, Config cfg)
        : BaseReader(queue), cfg_(cfg) {}

    ~DummyReader() override { disconnect(); }

    void connect() override {
        running_ = true;
        thread_  = std::thread(&DummyReader::loop, this);
        std::cout << "[DummyReader] Started at " << cfg_.hz << " Hz\n";
    }

    void disconnect() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        std::cout << "[DummyReader] Stopped.\n";
    }

    std::string name() const override { return "Dummy"; }
    bool isConnected() const override { return running_.load(); }

private:
    void loop() {
        const auto period = std::chrono::duration<double>(1.0 / cfg_.hz);
        int frame = 0;

        while (running_) {
            auto t0 = std::chrono::steady_clock::now();

            RawFrame raw;
            raw.frame_number = ++frame;
            raw.stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                t0.time_since_epoch()).count();

            // T-pose in right-hand coordinate system (Y=up, matches smplx)
            // positions are approximate human body dimensions (meters)
            // quat wxyz = identity [1,0,0,0]
            // T-pose matching xrobot_to_g1.json bone name convention
            // Coordinates in right-hand system (Z up), same as Python xrobot_utils output
            static const std::vector<std::pair<std::string, Eigen::Vector3d>> kTPose = {
                {"Pelvis",          { 0.0,  0.0,  1.0}},
                {"Left_Hip",        {-0.1,  0.0,  0.85}},
                {"Right_Hip",       { 0.1,  0.0,  0.85}},
                {"Spine3",          { 0.0,  0.0,  1.2}},
                {"Left_Knee",       {-0.1,  0.0,  0.5}},
                {"Right_Knee",      { 0.1,  0.0,  0.5}},
                {"Left_Foot",       {-0.1,  0.0,  0.1}},
                {"Right_Foot",      { 0.1,  0.0,  0.1}},
                {"Left_Shoulder",   {-0.2,  0.0,  1.3}},
                {"Right_Shoulder",  { 0.2,  0.0,  1.3}},
                {"Left_Elbow",      {-0.45, 0.0,  1.3}},
                {"Right_Elbow",     { 0.45, 0.0,  1.3}},
                {"Left_Wrist",      {-0.7,  0.0,  1.3}},
                {"Right_Wrist",     { 0.7,  0.0,  1.3}},
            };

            for (auto& [name, pos] : kTPose) {
                BodyData bd;
                bd.position = pos;
                bd.rot_wxyz = Eigen::Vector4d(1, 0, 0, 0);
                raw.body_data[name] = bd;
            }

            queue_.push(std::move(raw));

            std::this_thread::sleep_until(t0 + period);
        }
    }

    Config              cfg_;
    std::atomic<bool>   running_{false};
    std::thread         thread_;
};

} // namespace gmr