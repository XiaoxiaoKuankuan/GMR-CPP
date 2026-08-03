#include "g1_redis_receiver.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace gmr {

G1RedisReceiver::G1RedisReceiver(Config config)
    : config_(std::move(config)), parser_(30.0) {
    if (config_.port <= 0 || config_.port > 65535)
        throw std::runtime_error("G1 Redis port must be in [1,65535]");
    if (!std::isfinite(config_.poll_hz) || config_.poll_hz <= 0.0)
        throw std::runtime_error("G1 Redis poll_hz must be positive");
    if (config_.key.empty()) throw std::runtime_error("G1 Redis key is empty");
}

G1RedisReceiver::~G1RedisReceiver() {
    disconnect();
}

int64_t G1RedisReceiver::steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool G1RedisReceiver::connect_redis() {
    if (redis_) {
        redisFree(redis_);
        redis_ = nullptr;
    }
    timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 200000;
    redis_ = redisConnectWithTimeout(config_.host.c_str(), config_.port, timeout);
    if (!redis_ || redis_->err) {
        if (redis_) {
            redisFree(redis_);
            redis_ = nullptr;
        }
        return false;
    }
    if (config_.db != 0) {
        redisReply* reply = static_cast<redisReply*>(
            redisCommand(redis_, "SELECT %d", config_.db));
        const bool okay = reply && reply->type != REDIS_REPLY_ERROR;
        if (reply) freeReplyObject(reply);
        if (!okay) return false;
    }
    return true;
}

void G1RedisReceiver::connect() {
    if (connected_.exchange(true)) return;
    stop_ = false;
    thread_ = std::thread(&G1RedisReceiver::receiver_loop, this);
    if (config_.verbose) {
        std::cout << "[G1RedisReceiver] polling " << config_.host << ':'
                  << config_.port << '/' << config_.db << " key=" << config_.key
                  << " at " << config_.poll_hz << "Hz\n";
    }
}

void G1RedisReceiver::disconnect() {
    stop_ = true;
    condition_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (redis_) {
        redisFree(redis_);
        redis_ = nullptr;
    }
    if (connected_.exchange(false) && config_.verbose) {
        std::cout << "[G1RedisReceiver] stopped recv=" << frames_received()
                  << " invalid=" << frames_invalid() << '\n';
    }
}

double G1RedisReceiver::last_receive_age_ms() const {
    const int64_t last = last_receive_ns_.load();
    if (last <= 0) return std::numeric_limits<double>::infinity();
    return static_cast<double>(steady_now_ns() - last) * 1e-6;
}

bool G1RedisReceiver::get_next_frame(G1MotionFrame& frame,
                                     double timeout_seconds) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!condition_.wait_for(lock, std::chrono::duration<double>(timeout_seconds),
                             [&] { return stop_.load() || !queue_.empty(); }))
        return false;
    if (queue_.empty()) return false;
    frame = std::move(queue_.front());
    queue_.pop_front();
    return true;
}

void G1RedisReceiver::receiver_loop() {
    const auto period = std::chrono::duration<double>(1.0 / config_.poll_hz);
    auto next_tick = std::chrono::steady_clock::now();
    while (!stop_) {
        if (!redis_ && !connect_redis()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            continue;
        }
        redisReply* reply = static_cast<redisReply*>(
            redisCommand(redis_, "GET %s", config_.key.c_str()));
        if (!reply) {
            redisFree(redis_);
            redis_ = nullptr;
        } else {
            if (reply->type == REDIS_REPLY_STRING && reply->str && reply->len > 0) {
                const std::string payload(reply->str, static_cast<size_t>(reply->len));
                if (payload != last_payload_) {
                    if (parser_.load_json_frame(payload)) {
                        G1MotionFrame frame;
                        if (parser_.get_next_frame(frame)) {
                            std::lock_guard<std::mutex> lock(mutex_);
                            queue_.push_back(std::move(frame));
                            while (queue_.size() > config_.queue_size)
                                queue_.pop_front();
                            last_payload_ = payload;
                            last_receive_ns_ = steady_now_ns();
                            ++frames_received_;
                            condition_.notify_one();
                        }
                    } else {
                        ++frames_invalid_;
                        static auto last_warning = std::chrono::steady_clock::now() -
                                                   std::chrono::seconds(2);
                        const auto now = std::chrono::steady_clock::now();
                        if (config_.verbose && now - last_warning >=
                            std::chrono::seconds(1)) {
                            std::cerr << "[G1RedisReceiver] invalid frame: "
                                      << parser_.last_error() << '\n';
                            last_warning = now;
                        }
                    }
                }
            }
            freeReplyObject(reply);
        }
        next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        std::this_thread::sleep_until(next_tick);
        const auto now = std::chrono::steady_clock::now();
        if (now > next_tick + std::chrono::duration_cast<
                std::chrono::steady_clock::duration>(period))
            next_tick = now;
    }
}

}  // namespace gmr
