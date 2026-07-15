#pragma once
/**
 * gem_reader.hpp — fixed-size legacy GEM1 + SMPL-direct GEM2 UDP reader.
 */

#include "base_reader.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

namespace gmr {

enum class GemProtocol : int {
    Any = 0,
    Gem1 = 1,
    Gem2 = 2,
};

class GemReader final : public BaseReader {
public:
    struct Config {
        std::string bind_ip = "0.0.0.0";
        int         port    = 7001;
        bool        verbose = true;
        GemProtocol expected_protocol = GemProtocol::Any;
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
    GemProtocol currentProtocol() const { return current_protocol_.load(); }

    static const char* protocolName(GemProtocol protocol);
    static bool decodePacket(const uint8_t* data, size_t len,
                             RawFrame& out, GemProtocol& protocol);

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
    GemProtocol last_protocol_ = GemProtocol::Any;

    std::atomic<int64_t>  last_receive_ns_{0};
    std::atomic<uint64_t> packets_received_{0};
    std::atomic<uint64_t> packets_dropped_{0};
    std::atomic<uint64_t> packets_invalid_{0};
    std::atomic<GemProtocol> current_protocol_{GemProtocol::Any};
};

} // namespace gmr
