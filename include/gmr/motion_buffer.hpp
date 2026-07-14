#pragma once
#include "frame_queue.hpp"
#include "gmr_mink.hpp"
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <optional>
#include <functional>
#include <chrono>
#include <iostream>

namespace gmr {

struct ProcessedFrame {
    double          frame_time = 0.0;
    Eigen::VectorXd qpos;
    BodyMap         body_data;
};

class MotionBuffer {
public:
    using PreprocessFn = std::function<void(BodyMap&)>;

    explicit MotionBuffer(size_t max_frames,
                          double frame_timeout_sec = 0.02)
        : max_frames_(max_frames)
        , frame_timeout_(frame_timeout_sec)
    {}

    ~MotionBuffer() { stopAsync(); }

    void setPreprocessFn(PreprocessFn fn) { preprocess_fn_ = std::move(fn); }
    void setOffsetToGround(bool v) { offset_to_ground_ = v; }

    void seedSync(int n, FrameQueue& queue, gmr_mink::GMR* gmr) {
        for (int i = 0; i < n;) {
            RawFrame raw;
            if (!queue.pop(raw, frame_timeout_)) continue;
            auto pf = retarget(raw, gmr);
            if (pf) { append(*pf); ++i; }
        }
    }

    void startAsync(FrameQueue& queue, gmr_mink::GMR* gmr) {
        if (running_) return;
        running_ = true;
        thread_ = std::thread([this, &queue, gmr] {
            resetStats(queue);
            while (running_) {
                RawFrame raw;
                if (!queue.pop(raw, frame_timeout_)) continue;
                auto pf = retarget(raw, gmr);
                if (pf) {
                    append(*pf);
                    recordStats(queue);
                }
            }
        });
    }

    void stopAsync() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    void clear() {
        std::lock_guard<std::mutex> lk(mtx_);
        buf_.clear();
        dropped_ = 0;
    }

    size_t length() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return buf_.size();
    }

    int droppedCount() const { return dropped_.load(); }

    Eigen::VectorXd latestQpos() const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buf_.empty()) return {};
        return buf_.back().qpos;
    }

    BodyMap latestBodyData() const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buf_.empty()) return {};
        return buf_.back().body_data;
    }

    std::pair<Eigen::VectorXd, double> latestFrame() const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buf_.empty()) return {{}, 0.0};
        return {buf_.back().qpos, buf_.back().frame_time};
    }

    std::optional<ProcessedFrame> latestProcessedFrame() const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (buf_.empty()) return std::nullopt;
        return buf_.back();
    }

private:
    std::optional<ProcessedFrame> retarget(RawFrame& raw, gmr_mink::GMR* gmr) {
        if (preprocess_fn_) preprocess_fn_(raw.body_data);

        try {
            ProcessedFrame pf;
            if (raw.stamp_ns > 0) {
                pf.frame_time = raw.stamp_ns * 1e-9;
                static bool printed = false;
                if (!printed) {
                    std::printf("[MotionBuffer] using stamp_ns for frame_time\n");
                    printed = true;
                }
            } else {
                pf.frame_time = raw.frame_number / 100.0;
                static bool printed = false;
                if (!printed) {
                    std::printf("[MotionBuffer] using frame_number/100 for frame_time\n");
                    printed = true;
                }
            }
            pf.qpos      = gmr->retarget(raw.body_data, offset_to_ground_);
            pf.body_data = std::move(raw.body_data);
            return pf;
        } catch (const std::exception& e) {
            std::cerr << "[MotionBuffer] retarget error: " << e.what() << "\n";
            return std::nullopt;
        }
    }

    void resetStats(const FrameQueue& queue) {
        auto now = std::chrono::steady_clock::now();
        stats_t0_ = now;
        last_retarget_t_ = now;
        last_recv_total_ = queue.totalPushed();
        last_queue_drop_total_ = queue.totalDropped();
        retarget_count_ = 0;
        slow_count_ = 0;
        max_gap_ms_ = 0.0;
        seen_retarget_frame_ = false;
    }

    void recordStats(const FrameQueue& queue) {
        auto now = std::chrono::steady_clock::now();
        if (seen_retarget_frame_) {
            double gap_ms = std::chrono::duration<double, std::milli>(
                now - last_retarget_t_).count();
            if (gap_ms > 50.0) {
                slow_count_++;
                max_gap_ms_ = std::max(max_gap_ms_, gap_ms);
            }
        }
        last_retarget_t_ = now;
        seen_retarget_frame_ = true;
        retarget_count_++;

        double elapsed = std::chrono::duration<double>(now - stats_t0_).count();
        if (elapsed < 5.0) return;

        size_t recv_total = queue.totalPushed();
        size_t queue_drop_total = queue.totalDropped();
        size_t recv_delta = recv_total - last_recv_total_;
        size_t queue_drop_delta = queue_drop_total - last_queue_drop_total_;

        std::printf("[MotionBuffer] rates recv=%.1fHz retarget=%.1fHz "
                    "queue=%zu dropped=%zu slow>50ms=%d max_gap=%.1fms\n",
                    recv_delta / elapsed,
                    retarget_count_ / elapsed,
                    queue.size(),
                    queue_drop_delta,
                    slow_count_,
                    max_gap_ms_);

        stats_t0_ = now;
        last_recv_total_ = recv_total;
        last_queue_drop_total_ = queue_drop_total;
        retarget_count_ = 0;
        slow_count_ = 0;
        max_gap_ms_ = 0.0;
    }

    void append(ProcessedFrame pf) {
        std::lock_guard<std::mutex> lk(mtx_);
        buf_.push_back(std::move(pf));
        if (buf_.size() > max_frames_) {
            size_t ovf = buf_.size() - max_frames_;
            buf_.erase(buf_.begin(), buf_.begin() + ovf);
            dropped_ += int(ovf);
        }
    }

    size_t              max_frames_;
    double              frame_timeout_;
    PreprocessFn        preprocess_fn_;
    bool                offset_to_ground_ = false;  // default false，不影响mocap_server
    std::atomic<bool>   running_{false};
    std::atomic<int>    dropped_{0};
    std::thread         thread_;
    mutable std::mutex  mtx_;
    std::deque<ProcessedFrame> buf_;

    std::chrono::steady_clock::time_point stats_t0_;
    std::chrono::steady_clock::time_point last_retarget_t_;
    size_t last_recv_total_ = 0;
    size_t last_queue_drop_total_ = 0;
    int retarget_count_ = 0;
    int slow_count_ = 0;
    double max_gap_ms_ = 0.0;
    bool seen_retarget_frame_ = false;
};

} // namespace gmr