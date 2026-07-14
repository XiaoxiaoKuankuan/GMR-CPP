#pragma once
/**
 * gem_reader.hpp — fixed-protocol GEM1 UDP reader for GMR-CPP.
 */

#include "base_reader.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

namespace gmr {

class GemReader final : public BaseReader {
public:
    struct Config {
        std::string bind_ip = "0.0.0.0";
        int         port    = 7001;
        bool        verbose = true;
    };

    GemReader(FrameQueue& queue, Config cfg);
    ~GemReader() override;

    void connect() override;
    void disconnect() override;

    std::string name() const override { return "GEM"; }
    bool isConnected() const override { return connected_.load(); }

    bool hasReceivedFrame() const;
    double lastReceiveAgeMs() const;
    uint64_t packetsReceived() const { return packets_received_.load(); }
    uint64_t packetsDropped() const { return packets_dropped_.load(); }
    uint64_t packetsInvalid() const { return packets_invalid_.load(); }

private:
    void readerLoop();
    bool parsePacket(const uint8_t* data, size_t len, RawFrame& out);
    static int64_t steadyNowNs();

    Config cfg_;
    int sock_fd_ = -1;

    std::atomic<bool> connected_{false};
    std::atomic<bool> stop_flag_{false};
    std::thread reader_thread_;

    // Accessed only by reader_thread_.
    bool     have_sequence_ = false;
    uint32_t last_sequence_ = 0;

    std::atomic<int64_t>  last_receive_ns_{0};
    std::atomic<uint64_t> packets_received_{0};
    std::atomic<uint64_t> packets_dropped_{0};
    std::atomic<uint64_t> packets_invalid_{0};
};

} // namespace gmr
