#include "gmr/body_map.hpp"
#include "gmr/gmr_mink.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#ifndef BUMI3_REPO_ROOT
#define BUMI3_REPO_ROOT "."
#endif

namespace {

gmr::BodyData target(double z) {
    gmr::BodyData result;
    result.position = Eigen::Vector3d(0.0, 0.0, z);
    result.rot_wxyz = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);
    return result;
}

void expectNear(double actual, double expected, const std::string& label) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-12)
        throw std::runtime_error(
            label + ": expected " + std::to_string(expected) +
            ", got " + std::to_string(actual));
}

void checkTargets(const gmr::BodyMap& targets, double clearance) {
    expectNear(targets.at("left_foot").position.z(), clearance,
               "left foot z");
    expectNear(targets.at("right_foot").position.z(), clearance + 0.03,
               "right foot z");
    expectNear(targets.at("pelvis").position.z(), clearance + 0.57,
               "pelvis z");
}

}  // namespace

int main() {
    namespace fs = std::filesystem;
    const fs::path config_path =
        fs::temp_directory_path() / "gmr_ground_clearance_test.json";
    try {
        std::ofstream config(config_path);
        config << R"({
  "human_root_name": "pelvis",
  "ground_height": 0.0,
  "human_height_assumption": 1.8,
  "human_scale_table": {
    "pelvis": 1.0,
    "left_foot": 1.0,
    "right_foot": 1.0
  },
  "use_ik_match_table1": false,
  "use_ik_match_table2": false,
  "ik_match_table1": {},
  "ik_match_table2": {}
})";
        config.close();

        const std::string xml_path =
            std::string(BUMI3_REPO_ROOT) + "/assets/bumi3/mjcf/bumi3.xml";
        const gmr::BodyMap input = {
            {"left_foot", target(0.43)},
            {"right_foot", target(0.46)},
            {"pelvis", target(1.00)},
        };

        // GMR's unchanged default, used by the G1/E1 server presets.
        gmr_mink::GMR default_solver(
            xml_path, config_path.string(), 1.8, 1.0, false);
        default_solver.retarget(input, true);
        checkTargets(default_solver.getScaledHumanData(), 0.06);

        // BUMI3 grounded preset value.
        gmr_mink::GMR bumi3_solver(
            xml_path, config_path.string(), 1.8, 1.0, false);
        bumi3_solver.setGroundClearance(0.02);
        bumi3_solver.retarget(input, true);
        checkTargets(bumi3_solver.getScaledHumanData(), 0.02);

        // Jump mode uses one manually selected constant instead of per-frame
        // lowest-foot grounding. The same translation is applied to every target.
        gmr_mink::GMR fixed_offset_solver(
            xml_path, config_path.string(), 1.8, 1.0, false);
        fixed_offset_solver.setGroundOffset(0.41);
        fixed_offset_solver.retarget(input, false);
        checkTargets(fixed_offset_solver.getScaledHumanData(), 0.02);

        bool negative_rejected = false;
        try {
            bumi3_solver.setGroundClearance(-0.01);
        } catch (const std::runtime_error&) {
            negative_rejected = true;
        }
        if (!negative_rejected)
            throw std::runtime_error("negative clearance was accepted");

        bool nonfinite_rejected = false;
        try {
            bumi3_solver.setGroundClearance(
                std::numeric_limits<double>::infinity());
        } catch (const std::runtime_error&) {
            nonfinite_rejected = true;
        }
        if (!nonfinite_rejected)
            throw std::runtime_error("non-finite clearance was accepted");

        fs::remove(config_path);
        std::cout << "ground_clearance_test: PASS "
                     "default=0.06 bumi3=0.02 fixed_offset=0.41\n";
        return 0;
    } catch (const std::exception& error) {
        std::error_code ignored;
        fs::remove(config_path, ignored);
        std::cerr << "ground_clearance_test: FAIL: " << error.what() << "\n";
        return 1;
    }
}
