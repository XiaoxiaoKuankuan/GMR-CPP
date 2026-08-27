/**
 * Real-time BUMI3 qpos viewer used by the GENMO safety bridge.
 *
 * stdin is a stream of atomic little-endian float32 qpos[28] records.  The
 * reader drains stale records before every render so the window follows the
 * bridge's current 50 Hz reference instead of replaying a delayed FIFO.
 */

#include "gmr/mujoco_viewer.hpp"

#include <Eigen/Dense>

#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

std::atomic<bool> g_stop{false};

namespace {

constexpr int kQposDim = 28;
constexpr size_t kFrameBytes = kQposDim * sizeof(float);
constexpr uint8_t kReadyByte = 'V';

struct Config {
    std::string xml;
    int width = 640;
    int height = 480;
    std::string follow_body = "base_link";
};

Config parseArgs(int argc, char** argv) {
    namespace fs = std::filesystem;
    Config cfg;
    const fs::path executable = fs::canonical(argv[0]);
    const fs::path root = executable.parent_path().parent_path();
    cfg.xml = (root / "assets/bumi3/mjcf/bumi3.xml").string();
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        auto next = [&]() -> std::string {
            if (++index >= argc)
                throw std::runtime_error("missing value for " + argument);
            return argv[index];
        };
        if (argument == "--xml") cfg.xml = next();
        else if (argument == "--viewer-width") cfg.width = std::stoi(next());
        else if (argument == "--viewer-height") cfg.height = std::stoi(next());
        else if (argument == "--follow-body") cfg.follow_body = next();
        else if (argument == "--help" || argument == "-h") {
            std::cerr << "Usage: " << argv[0]
                      << " [--xml PATH] [--viewer-width PX]"
                      << " [--viewer-height PX] [--follow-body NAME]\n";
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + argument);
        }
    }
    if (!fs::is_regular_file(cfg.xml))
        throw std::runtime_error("robot XML not found: " + cfg.xml);
    if (cfg.width < 160 || cfg.height < 120)
        throw std::runtime_error("viewer dimensions must be at least 160x120");
    if (cfg.follow_body.empty())
        throw std::runtime_error("follow body must not be empty");
    return cfg;
}

Eigen::VectorXd decodeFrame(const std::array<uint8_t, kFrameBytes>& bytes) {
    Eigen::VectorXd result(kQposDim);
    for (int index = 0; index < kQposDim; ++index) {
        float value = 0.0F;
        std::memcpy(&value, bytes.data() + index * sizeof(float), sizeof(float));
        if (!std::isfinite(value))
            throw std::runtime_error("received non-finite qpos");
        result[index] = static_cast<double>(value);
    }
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Config cfg = parseArgs(argc, argv);
        gmr::MujocoViewer viewer(
            cfg.xml, cfg.width, cfg.height, cfg.follow_body);

        if (::write(STDOUT_FILENO, &kReadyByte, 1) != 1)
            throw std::runtime_error("failed to send viewer readiness byte");

        const int flags = ::fcntl(STDIN_FILENO, F_GETFL, 0);
        if (flags < 0 || ::fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) < 0)
            throw std::runtime_error("failed to make viewer input non-blocking");

        std::array<uint8_t, kFrameBytes> bytes{};
        Eigen::VectorXd latest;
        bool input_open = true;
        while (!g_stop && input_open) {
            pollfd descriptor{STDIN_FILENO, POLLIN, 0};
            const int ready = ::poll(&descriptor, 1, 16);
            if (ready < 0 && errno != EINTR)
                throw std::runtime_error("viewer input poll failed");

            if (ready > 0 && (descriptor.revents & (POLLIN | POLLHUP))) {
                while (true) {
                    const ssize_t count = ::read(
                        STDIN_FILENO, bytes.data(), bytes.size());
                    if (count == static_cast<ssize_t>(bytes.size())) {
                        // Drain every queued record and retain only the newest.
                        latest = decodeFrame(bytes);
                        continue;
                    }
                    if (count == 0) {
                        input_open = false;
                        break;
                    }
                    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                        break;
                    if (count < 0 && errno == EINTR) continue;
                    if (count > 0)
                        throw std::runtime_error("truncated qpos viewer frame");
                    if (count < 0)
                        throw std::runtime_error("qpos viewer read failed");
                }
            }

            if (latest.size() == kQposDim && !viewer.render(latest)) break;
        }
        std::cerr << "[BUMI3 qpos viewer] stopped\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "[BUMI3 qpos viewer fatal] " << error.what() << "\n";
        return 1;
    }
}
