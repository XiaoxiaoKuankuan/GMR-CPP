#include "smplx_reader.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

constexpr size_t   kHeaderBytes = 20;
constexpr size_t   kTargetCount = 14;
constexpr size_t   kFloatsPerTarget = 7;
constexpr size_t   kPacketBytes =
    kHeaderBytes + kTargetCount * kFloatsPerTarget * sizeof(float);
constexpr uint16_t kVersion = 1;
constexpr double   kMaxAbsPositionM = 20.0;

static_assert(kPacketBytes == 412, "SMP1 packet must be exactly 412 bytes");

const std::array<const char*, kTargetCount> kTargetNames = {
    "pelvis",
    "spine3",
    "left_hip",
    "right_hip",
    "left_knee",
    "right_knee",
    "left_foot",
    "right_foot",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
};

uint16_t readU16LE(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8U);
}

uint32_t readU32LE(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8U) |
           (static_cast<uint32_t>(p[2]) << 16U) |
           (static_cast<uint32_t>(p[3]) << 24U);
}

uint64_t readU64LE(const uint8_t* p) {
    return static_cast<uint64_t>(readU32LE(p)) |
           (static_cast<uint64_t>(readU32LE(p + 4)) << 32U);
}

float readFloatLE(const uint8_t* p) {
    const uint32_t bits = readU32LE(p);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

bool finitePosition(double value) {
    return std::isfinite(value) && std::abs(value) <= kMaxAbsPositionM;
}

} // namespace

namespace gmr {

SmplxReader::SmplxReader(FrameQueue& queue, Config cfg)
    : BaseReader(queue), cfg_(std::move(cfg)) {}

SmplxReader::~SmplxReader() {
    disconnect();
}

int64_t SmplxReader::steadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void SmplxReader::connect() {
    if (connected_.load()) return;
    if (cfg_.port < 1 || cfg_.port > 65535)
        throw std::runtime_error("[SmplxReader] port must be in [1, 65535]");

    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0)
        throw std::runtime_error("[SmplxReader] socket() failed: " +
                                 std::string(std::strerror(errno)));

    int one = 1;
    if (::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR,
                     &one, sizeof(one)) < 0) {
        std::cerr << "[SmplxReader] warning: SO_REUSEADDR failed: "
                  << std::strerror(errno) << "\n";
    }
    int receive_buffer = 4 * 1024 * 1024;
    if (::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF,
                     &receive_buffer, sizeof(receive_buffer)) < 0) {
        std::cerr << "[SmplxReader] warning: SO_RCVBUF failed: "
                  << std::strerror(errno) << "\n";
    }
    timeval timeout{};
    timeout.tv_usec = 100000;
    if (::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO,
                     &timeout, sizeof(timeout)) < 0) {
        std::cerr << "[SmplxReader] warning: SO_RCVTIMEO failed: "
                  << std::strerror(errno) << "\n";
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(cfg_.port));
    if (cfg_.bind_ip.empty() || cfg_.bind_ip == "0.0.0.0") {
        address.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (::inet_pton(AF_INET, cfg_.bind_ip.c_str(),
                           &address.sin_addr) != 1) {
        ::close(sock_fd_);
        sock_fd_ = -1;
        throw std::runtime_error("[SmplxReader] invalid IPv4 --bind: " +
                                 cfg_.bind_ip);
    }
    if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&address),
               sizeof(address)) < 0) {
        const std::string reason = std::strerror(errno);
        ::close(sock_fd_);
        sock_fd_ = -1;
        throw std::runtime_error("[SmplxReader] bind " + cfg_.bind_ip + ":" +
                                 std::to_string(cfg_.port) + " failed: " + reason);
    }

    have_sequence_ = false;
    last_sequence_ = 0;
    last_receive_ns_ = 0;
    packets_received_ = 0;
    packets_dropped_ = 0;
    packets_invalid_ = 0;
    stop_flag_ = false;
    connected_ = true;
    reader_thread_ = std::thread(&SmplxReader::readerLoop, this);
    if (cfg_.verbose) {
        std::printf("[SmplxReader] listening on %s:%d SMP1 packet=%zu bytes\n",
                    cfg_.bind_ip.c_str(), cfg_.port, kPacketBytes);
    }
}

void SmplxReader::disconnect() {
    stop_flag_ = true;
    if (sock_fd_ >= 0) ::shutdown(sock_fd_, SHUT_RDWR);
    if (reader_thread_.joinable()) reader_thread_.join();
    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }
    if (connected_.exchange(false) && cfg_.verbose) {
        std::printf(
            "[SmplxReader] stopped recv=%llu drop=%llu invalid=%llu queue_drop=%zu\n",
            static_cast<unsigned long long>(packetsReceived()),
            static_cast<unsigned long long>(packetsDropped()),
            static_cast<unsigned long long>(packetsInvalid()),
            queue_.totalDropped());
    }
}

bool SmplxReader::hasReceivedFrame() const {
    return last_receive_ns_.load() > 0;
}

double SmplxReader::lastReceiveAgeMs() const {
    const int64_t last = last_receive_ns_.load();
    if (last <= 0) return std::numeric_limits<double>::infinity();
    return std::max(0.0, static_cast<double>(steadyNowNs() - last) * 1e-6);
}

bool SmplxReader::decodePacket(const uint8_t* data, size_t len, RawFrame& out) {
    if (data == nullptr || len != kPacketBytes) return false;
    if (std::memcmp(data, "SMP1", 4) != 0 ||
        readU16LE(data + 4) != kVersion ||
        readU16LE(data + 6) != kTargetCount) {
        return false;
    }

    RawFrame parsed;
    parsed.frame_number = readU32LE(data + 8);
    const uint64_t source_stamp_ns = readU64LE(data + 12);
    (void)source_stamp_ns;

    const uint8_t* cursor = data + kHeaderBytes;
    for (size_t target = 0; target < kTargetCount; ++target) {
        std::array<float, kFloatsPerTarget> values{};
        for (float& value : values) {
            value = readFloatLE(cursor);
            cursor += sizeof(float);
            if (!std::isfinite(value)) return false;
        }
        if (!finitePosition(values[0]) || !finitePosition(values[1]) ||
            !finitePosition(values[2])) {
            return false;
        }
        const double norm = std::sqrt(
            static_cast<double>(values[3]) * values[3] +
            static_cast<double>(values[4]) * values[4] +
            static_cast<double>(values[5]) * values[5] +
            static_cast<double>(values[6]) * values[6]);
        if (!std::isfinite(norm) || norm < 1e-6) return false;

        BodyData pose;
        pose.position = Eigen::Vector3d(values[0], values[1], values[2]);
        pose.rot_wxyz = Eigen::Vector4d(
            values[3] / norm, values[4] / norm,
            values[5] / norm, values[6] / norm);
        parsed.body_data.emplace(kTargetNames[target], std::move(pose));
    }
    out = std::move(parsed);
    return true;
}

bool SmplxReader::parsePacket(const uint8_t* data, size_t len, RawFrame& out) {
    RawFrame parsed;
    if (!decodePacket(data, len, parsed)) return false;
    const uint32_t sequence = parsed.frame_number;
    if (have_sequence_) {
        const uint32_t delta = sequence - last_sequence_;
        if (delta == 0 || delta >= 0x80000000U) return false;
        if (delta > 1)
            packets_dropped_.fetch_add(static_cast<uint64_t>(delta - 1));
    }
    have_sequence_ = true;
    last_sequence_ = sequence;
    parsed.stamp_ns = steadyNowNs();
    out = std::move(parsed);
    return true;
}

void SmplxReader::readerLoop() {
    std::array<uint8_t, 2048> buffer{};
    auto diagnostic_start = std::chrono::steady_clock::now();
    auto last_error_log = diagnostic_start - std::chrono::seconds(5);
    uint64_t interval_received = 0;
    size_t previous_queue_drops = queue_.totalDropped();

    while (!stop_flag_.load()) {
        const ssize_t bytes = ::recvfrom(
            sock_fd_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
        if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;
            const auto now = std::chrono::steady_clock::now();
            if (!stop_flag_.load() &&
                now - last_error_log >= std::chrono::seconds(1)) {
                std::cerr << "[SmplxReader] recvfrom failed: "
                          << std::strerror(errno) << "\n";
                last_error_log = now;
            }
            continue;
        }
        if (stop_flag_.load()) break;

        RawFrame frame;
        if (!parsePacket(buffer.data(), static_cast<size_t>(bytes), frame)) {
            packets_invalid_.fetch_add(1);
        } else {
            last_receive_ns_ = frame.stamp_ns;
            queue_.push(std::move(frame));
            packets_received_.fetch_add(1);
            ++interval_received;
        }

        const auto now = std::chrono::steady_clock::now();
        const double elapsed =
            std::chrono::duration<double>(now - diagnostic_start).count();
        if (cfg_.verbose && elapsed >= 5.0) {
            const size_t queue_drops = queue_.totalDropped();
            std::printf(
                "[SmplxReader] recv=%.1fHz total=%llu drop=%llu invalid=%llu "
                "queue_drop=%zu last_age=%.1fms\n",
                interval_received / elapsed,
                static_cast<unsigned long long>(packetsReceived()),
                static_cast<unsigned long long>(packetsDropped()),
                static_cast<unsigned long long>(packetsInvalid()),
                queue_drops - previous_queue_drops,
                lastReceiveAgeMs());
            diagnostic_start = now;
            interval_received = 0;
            previous_queue_drops = queue_drops;
        }
    }
}

} // namespace gmr
