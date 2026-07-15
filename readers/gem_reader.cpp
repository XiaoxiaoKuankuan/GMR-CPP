#include "gem_reader.hpp"

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
constexpr size_t   kBoneCount   = 14;
constexpr size_t   kFloatsPerBone = 7;
constexpr size_t   kPacketBytes = kHeaderBytes + kBoneCount * kFloatsPerBone * 4;
constexpr uint16_t kGem1Version = 1;
constexpr uint16_t kGem2Version = 2;
constexpr double   kMaxAbsPositionM = 20.0;

static_assert(kPacketBytes == 412, "GEM1/GEM2 packet must be exactly 412 bytes");

const std::array<const char*, kBoneCount> kGem1Names = {
    "Pelvis",
    "Chest",
    "Left_UpperLeg",
    "Right_UpperLeg",
    "Left_LowerLeg",
    "Right_LowerLeg",
    "Left_Foot",
    "Right_Foot",
    "Left_UpperArm",
    "Right_UpperArm",
    "Left_Forearm",
    "Right_Forearm",
    "Left_Hand",
    "Right_Hand",
};

const std::array<const char*, kBoneCount> kGem2Names = {
    "SMPL_Pelvis",
    "SMPL_Chest",
    "SMPL_LeftHip",
    "SMPL_RightHip",
    "SMPL_LeftKnee",
    "SMPL_RightKnee",
    "SMPL_LeftAnkle",
    "SMPL_RightAnkle",
    "SMPL_LeftShoulder",
    "SMPL_RightShoulder",
    "SMPL_LeftElbow",
    "SMPL_RightElbow",
    "SMPL_LeftWrist",
    "SMPL_RightWrist",
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

bool finiteAndBounded(double value) {
    return std::isfinite(value) && std::abs(value) <= kMaxAbsPositionM;
}

} // namespace

namespace gmr {

const char* GemReader::protocolName(GemProtocol protocol) {
    switch (protocol) {
    case GemProtocol::Gem1: return "GEM1";
    case GemProtocol::Gem2: return "GEM2";
    case GemProtocol::Any: return "auto";
    }
    return "unknown";
}

GemReader::GemReader(FrameQueue& queue, Config cfg)
    : BaseReader(queue), cfg_(std::move(cfg)) {}

GemReader::~GemReader() {
    disconnect();
}

int64_t GemReader::steadyNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void GemReader::connect() {
    if (connected_.load()) return;
    if (cfg_.port < 1 || cfg_.port > 65535)
        throw std::runtime_error("[GemReader] port must be in [1, 65535]");

    sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd_ < 0)
        throw std::runtime_error("[GemReader] socket() failed: " +
                                 std::string(std::strerror(errno)));

    int one = 1;
    if (::setsockopt(sock_fd_, SOL_SOCKET, SO_REUSEADDR,
                     &one, sizeof(one)) < 0) {
        std::cerr << "[GemReader] warning: SO_REUSEADDR failed: "
                  << std::strerror(errno) << "\n";
    }

    int receive_buffer = 4 * 1024 * 1024;
    if (::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVBUF,
                     &receive_buffer, sizeof(receive_buffer)) < 0) {
        std::cerr << "[GemReader] warning: SO_RCVBUF failed: "
                  << std::strerror(errno) << "\n";
    }

    timeval timeout{};
    timeout.tv_usec = 100000;
    if (::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO,
                     &timeout, sizeof(timeout)) < 0) {
        std::cerr << "[GemReader] warning: SO_RCVTIMEO failed: "
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
        throw std::runtime_error("[GemReader] invalid IPv4 --bind: " +
                                 cfg_.bind_ip);
    }

    if (::bind(sock_fd_, reinterpret_cast<sockaddr*>(&address),
               sizeof(address)) < 0) {
        const std::string reason = std::strerror(errno);
        ::close(sock_fd_);
        sock_fd_ = -1;
        throw std::runtime_error("[GemReader] bind " + cfg_.bind_ip + ":" +
                                 std::to_string(cfg_.port) + " failed: " + reason);
    }

    have_sequence_ = false;
    last_sequence_ = 0;
    last_protocol_ = GemProtocol::Any;
    current_protocol_ = GemProtocol::Any;
    last_receive_ns_ = 0;
    packets_received_ = 0;
    packets_dropped_ = 0;
    packets_invalid_ = 0;
    stop_flag_ = false;
    connected_ = true;
    reader_thread_ = std::thread(&GemReader::readerLoop, this);

    if (cfg_.verbose) {
        std::printf("[GemReader] listening on %s:%d packet=%zu bytes\n",
                    cfg_.bind_ip.c_str(), cfg_.port, kPacketBytes);
    }
}

void GemReader::disconnect() {
    stop_flag_ = true;
    if (sock_fd_ >= 0)
        ::shutdown(sock_fd_, SHUT_RDWR);

    if (reader_thread_.joinable())
        reader_thread_.join();

    if (sock_fd_ >= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
    }

    if (connected_.exchange(false) && cfg_.verbose) {
        std::printf(
            "[GemReader] stopped recv=%llu drop=%llu invalid=%llu queue_drop=%zu\n",
            static_cast<unsigned long long>(packetsReceived()),
            static_cast<unsigned long long>(packetsDropped()),
            static_cast<unsigned long long>(packetsInvalid()),
            queue_.totalDropped());
    }
}

bool GemReader::hasReceivedFrame() const {
    return last_receive_ns_.load() > 0;
}

double GemReader::lastReceiveAgeMs() const {
    const int64_t last = last_receive_ns_.load();
    if (last <= 0) return std::numeric_limits<double>::infinity();
    return std::max(0.0, static_cast<double>(steadyNowNs() - last) * 1e-6);
}

bool GemReader::parsePacket(const uint8_t* data, size_t len, RawFrame& out) {
    RawFrame parsed;
    GemProtocol protocol = GemProtocol::Any;
    if (!decodePacket(data, len, parsed, protocol)) return false;
    if (cfg_.expected_protocol != GemProtocol::Any &&
        protocol != cfg_.expected_protocol) {
        return false;
    }

    const uint32_t sequence = readU32LE(data + 8);
    if (protocol != last_protocol_) {
        have_sequence_ = false;
        last_protocol_ = protocol;
        current_protocol_ = protocol;
        if (cfg_.verbose) {
            std::printf(
                "[GemReader] protocol=%s version=%u targets=%s\n",
                protocolName(protocol),
                static_cast<unsigned>(readU16LE(data + 4)),
                protocol == GemProtocol::Gem2 ? "SMPL_* joint centers" :
                                                "legacy segment names");
        }
    }

    // RFC-1982-style serial arithmetic: [1, 2^31-1] is forward, including
    // uint32 wrap; zero is duplicate, and the upper half-range is old/reordered.
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

bool GemReader::decodePacket(const uint8_t* data, size_t len,
                             RawFrame& out, GemProtocol& protocol) {
    if (data == nullptr || len != kPacketBytes) return false;
    const uint16_t version = readU16LE(data + 4);
    const std::array<const char*, kBoneCount>* names = nullptr;
    if (std::memcmp(data, "GEM1", 4) == 0 && version == kGem1Version) {
        protocol = GemProtocol::Gem1;
        names = &kGem1Names;
    } else if (std::memcmp(data, "GEM2", 4) == 0 && version == kGem2Version) {
        protocol = GemProtocol::Gem2;
        names = &kGem2Names;
    } else {
        return false;
    }
    if (readU16LE(data + 6) != kBoneCount) return false;

    const uint32_t sequence = readU32LE(data + 8);
    const uint64_t source_stamp_ns = readU64LE(data + 12);
    (void)source_stamp_ns; // Sender and receiver monotonic epochs are unrelated.

    RawFrame parsed;
    parsed.frame_number = sequence;

    const uint8_t* cursor = data + kHeaderBytes;
    for (size_t bone = 0; bone < kBoneCount; ++bone) {
        std::array<float, kFloatsPerBone> values{};
        for (size_t field = 0; field < kFloatsPerBone; ++field) {
            values[field] = readFloatLE(cursor);
            cursor += 4;
            if (!std::isfinite(values[field])) return false;
        }

        if (!finiteAndBounded(values[0]) ||
            !finiteAndBounded(values[1]) ||
            !finiteAndBounded(values[2])) {
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
        parsed.body_data.emplace((*names)[bone], std::move(pose));
    }
    out = std::move(parsed);
    return true;
}

void GemReader::readerLoop() {
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
            if (!stop_flag_.load() && now - last_error_log >= std::chrono::seconds(1)) {
                std::cerr << "[GemReader] recvfrom failed: "
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
                "[GemReader] recv=%.1fHz total=%llu drop=%llu invalid=%llu "
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
