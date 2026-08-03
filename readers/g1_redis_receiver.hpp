#pragma once

#include "g1_motion_reader.hpp"

#include <hiredis/hiredis.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace gmr {

class G1RedisReceiver {
public:
    struct Config {
        std::string host = "127.0.0.1";
        int port = 6379;
        int db = 0;
        std::string key = "omg_online_frame_g1";
        double poll_hz = 60.0;
        size_t queue_size = 120;
        bool verbose = true;
    };

    explicit G1RedisReceiver(Config config);
    ~G1RedisReceiver();

    void connect();
    void disconnect();
    bool get_next_frame(G1MotionFrame& frame, double timeout_seconds);

    bool has_received_frame() const { return last_receive_ns_.load() > 0; }
    double last_receive_age_ms() const;
    uint64_t frames_received() const { return frames_received_.load(); }
    uint64_t frames_invalid() const { return frames_invalid_.load(); }

private:
    void receiver_loop();
    bool connect_redis();
    static int64_t steady_now_ns();

    Config config_;
    redisContext* redis_ = nullptr;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<bool> connected_{false};
    std::atomic<int64_t> last_receive_ns_{0};
    std::atomic<uint64_t> frames_received_{0};
    std::atomic<uint64_t> frames_invalid_{0};
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<G1MotionFrame> queue_;
    std::string last_payload_;
    G1MotionReader parser_;
};

}  // namespace gmr
