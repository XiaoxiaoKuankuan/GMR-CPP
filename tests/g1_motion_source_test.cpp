#include "adapters/g1_motion_adapter.hpp"
#include "gmr/npy_io.hpp"
#include "readers/g1_motion_reader.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef G1_BUMI3_REPO_ROOT
#define G1_BUMI3_REPO_ROOT "."
#endif

int main() {
    try {
        const std::filesystem::path temporary =
            std::filesystem::temp_directory_path() / "g1_motion_source_test.npy";
        std::vector<double> qpos(2 * 36, 0.0);
        for (size_t row = 0; row < 2; ++row) {
            qpos[row * 36 + 2] = 0.793;
            qpos[row * 36 + 3] = 1.0;
            qpos[row * 36 + 7 + 3] = 0.1 * static_cast<double>(row);
        }
        gmr::npy::save2D(temporary.string(), qpos, 2, 36);
        gmr::G1MotionReader reader(30.0);
        if (!reader.load_npy(temporary.string()))
            throw std::runtime_error(reader.last_error());
        gmr::G1MotionFrame first, second;
        if (!reader.get_next_frame(first) || !reader.get_next_frame(second) ||
            reader.get_next_frame(first))
            throw std::runtime_error("NPY frame iteration failed");
        if (std::abs(second.timestamp - 1.0 / 30.0) > 1e-12 ||
            second.joint_positions.size() != 29 ||
            std::abs(second.joint_positions[3] - 0.1) > 1e-12)
            throw std::runtime_error("NPY frame values differ");

        const std::string json =
            "{\"timestamp\":1.25,\"root_pos\":[0,0,0.793],"
            "\"root_quat\":[2,0,0,0],\"joints\":["
            "0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}";
        if (!reader.load_json_frame(json) || !reader.get_next_frame(first))
            throw std::runtime_error(reader.last_error());
        if (std::abs(first.root_rotation.norm() - 1.0) > 1e-12)
            throw std::runtime_error("JSON quaternion was not normalized");

        const std::string model = std::string(G1_BUMI3_REPO_ROOT) +
            "/assets/unitree_g1/g1_mocap_29dof.xml";
        gmr::G1MotionAdapter adapter(model);
        const gmr::BodyMap bodies = adapter.to_body_map(first);
        if (bodies.size() != gmr::G1MotionAdapter::source_body_names().size())
            throw std::runtime_error("adapter body count mismatch");
        for (const std::string& name : gmr::G1MotionAdapter::source_body_names())
            if (!bodies.count(name) || !bodies.at(name).position.allFinite())
                throw std::runtime_error("adapter missing/non-finite body: " + name);
        if (adapter.source_qpos().size() != 36)
            throw std::runtime_error("adapter source qpos is not 36D");
        std::filesystem::remove(temporary);
        std::cout << "g1_motion_source_test: PASS\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "g1_motion_source_test: FAIL: " << error.what() << '\n';
        return 1;
    }
}
