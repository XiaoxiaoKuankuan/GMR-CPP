#include "smplx_reader.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>

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
    std::memcpy(&bits, &value, sizeof(bits));
    writeU32(output, bits);
}

std::array<std::uint8_t, kPacketBytes> makePacket() {
    std::array<std::uint8_t, kPacketBytes> packet{};
    std::memcpy(packet.data(), "SMP1", 4);
    writeU16(packet.data() + 4, 1);
    writeU16(packet.data() + 6, 14);
    writeU32(packet.data() + 8, 73);
    writeU64(packet.data() + 12, 987654321);
    std::uint8_t* cursor = packet.data() + 20;
    for (int item = 0; item < 14; ++item) {
        const std::array<float, 7> values = {
            0.1F * item, -0.05F * item, 0.8F + 0.01F * item,
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
        auto packet = makePacket();
        gmr::RawFrame frame;
        require(gmr::SmplxReader::decodePacket(
                    packet.data(), packet.size(), frame),
                "valid SMP1 packet did not decode");
        require(frame.frame_number == 73, "wrong SMP1 sequence");
        require(frame.body_data.size() == 14, "wrong target count");
        require(frame.body_data.count("pelvis") == 1, "pelvis missing");
        require(frame.body_data.count("spine3") == 1, "spine3 missing");
        require(frame.body_data.count("left_foot") == 1, "left_foot missing");
        require(frame.body_data.count("SMPL_Pelvis") == 0,
                "SMP1 leaked GEM2 names");
        require(std::abs(frame.body_data.at("pelvis").rot_wxyz[0] - 1.0) < 1e-12,
                "quaternion was not normalized");

        auto invalid_magic = packet;
        std::memcpy(invalid_magic.data(), "GEM2", 4);
        require(!gmr::SmplxReader::decodePacket(
                    invalid_magic.data(), invalid_magic.size(), frame),
                "SMP1 reader accepted GEM2 magic");
        auto invalid_count = packet;
        writeU16(invalid_count.data() + 6, 13);
        require(!gmr::SmplxReader::decodePacket(
                    invalid_count.data(), invalid_count.size(), frame),
                "SMP1 reader accepted wrong item count");
        require(!gmr::SmplxReader::decodePacket(
                    packet.data(), packet.size() - 1, frame),
                "SMP1 reader accepted short packet");

        std::cout << "SMP1 parser self-test passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "SMP1 parser self-test failed: " << error.what() << "\n";
        return 1;
    }
}
