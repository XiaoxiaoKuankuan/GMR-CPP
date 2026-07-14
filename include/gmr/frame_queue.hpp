#pragma once
/**
 * frame_queue.hpp — Thread-safe raw frame queue
 *
 * Readers push RawFrames; MotionBuffer pops them.
 */

#include "body_map.hpp"
#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <condition_variable>

namespace gmr {

struct RawFrame {
    uint32_t frame_number = 0;
    int64_t stamp_ns     = 0;
    BodyMap body_data;
};

class FrameQueue {
public:
    explicit FrameQueue(size_t max_size = 300) : max_size_(max_size) {}

    void push(RawFrame f) {
        std::lock_guard<std::mutex> lk(mtx_);
        q_.push_back(std::move(f));
        total_pushed_++;
        if (q_.size() > max_size_) {
            q_.pop_front();
            total_dropped_++;
        }
        cv_.notify_one();
    }

    // Returns false on timeout
    bool pop(RawFrame& out, double timeout_sec) {
        std::unique_lock<std::mutex> lk(mtx_);
        auto dl = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(timeout_sec);
        if (!cv_.wait_until(lk, dl, [&] { return !q_.empty(); }))
            return false;
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return q_.size();
    }

    size_t totalPushed() const { return total_pushed_.load(); }
    size_t totalDropped() const { return total_dropped_.load(); }

private:
    size_t                  max_size_;
    std::deque<RawFrame>    q_;
    mutable std::mutex      mtx_;
    std::condition_variable cv_;
    std::atomic<size_t>     total_pushed_{0};
    std::atomic<size_t>     total_dropped_{0};
};

} // namespace gmr
