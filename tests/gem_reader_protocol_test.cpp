#include "gem_reader.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr std::size_t kPacketBytes = 412;

void writeU16(std::uint8_t* output, std::uint16_t value) {
    output[0] = static_cast<std::uint8_t>(value & 0xffU);
    output[1] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void writeU32(std::uint8_t* output, std::uint32_t value) {
    for (int index = 0; index < 4; ++index)
        output[index] = static_cast<std::uint8_t>(value >> (8U * index));
}

void writeU64(std::uint8_t* output, std::uint64_t value) {
    for (int index = 0; index < 8; ++index)
        output[index] = static_cast<std::uint8_t>(value >> (8U * index));
}

void writeFloat(std::uint8_t* output, float value) {
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    writeU32(output, bits);
}

std::array<std::uint8_t, kPacketBytes> makePacket(
    const char magic[4], std::uint16_t version) {
    std::array<std::uint8_t, kPacketBytes> packet{};
    std::memcpy(packet.data(), magic, 4);
    writeU16(packet.data() + 4, version);
    writeU16(packet.data() + 6, 14);
    writeU32(packet.data() + 8, 42);
    writeU64(packet.data() + 12, 123456789);
    std::uint8_t* cursor = packet.data() + 20;
    for (int item = 0; item < 14; ++item) {
        const std::array<float, 7> values = {
            0.1F * item, -0.2F * item, 0.8F + 0.01F * item,
            2.0F, 0.0F, 0.0F, 0.0F,
        };
        for (float value : values) {
            writeFloat(cursor, value);
            cursor += 4;
        }
    }
    return packet;
}

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

} // namespace

int main() {
    try {
        gmr::RawFrame frame;
        gmr::GemProtocol protocol = gmr::GemProtocol::Any;

        auto gem1 = makePacket("GEM1", 1);
        require(gmr::GemReader::decodePacket(
            gem1.data(), gem1.size(), frame, protocol), "GEM1 did not decode");
        require(protocol == gmr::GemProtocol::Gem1, "wrong GEM1 protocol");
        require(frame.frame_number == 42, "wrong GEM1 sequence");
        require(frame.body_data.size() == 14, "wrong GEM1 item count");
        require(frame.body_data.count("Pelvis") == 1, "GEM1 Pelvis missing");
        require(frame.body_data.count("SMPL_Pelvis") == 0,
                "GEM1 used GEM2 names");
        require(std::abs(frame.body_data.at("Pelvis").rot_wxyz[0] - 1.0) < 1e-12,
                "GEM1 quaternion was not normalized");

        auto gem2 = makePacket("GEM2", 2);
        require(gmr::GemReader::decodePacket(
            gem2.data(), gem2.size(), frame, protocol), "GEM2 did not decode");
        require(protocol == gmr::GemProtocol::Gem2, "wrong GEM2 protocol");
        require(frame.body_data.size() == 14, "wrong GEM2 item count");
        require(frame.body_data.count("SMPL_Pelvis") == 1,
                "GEM2 SMPL_Pelvis missing");
        require(frame.body_data.count("SMPL_LeftShoulder") == 1,
                "GEM2 SMPL_LeftShoulder missing");
        require(frame.body_data.count("Left_UpperArm") == 0,
                "GEM2 leaked GEM1 names");

        auto wrong_version = makePacket("GEM2", 1);
        require(!gmr::GemReader::decodePacket(
            wrong_version.data(), wrong_version.size(), frame, protocol),
            "GEM2 accepted version 1");
        require(!gmr::GemReader::decodePacket(
            gem2.data(), gem2.size() - 1, frame, protocol),
            "GEM2 accepted a short packet");

        std::cout << "GEM1/GEM2 parser self-test passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "GEM parser self-test failed: " << error.what() << "\n";
        return 1;
    }
}
