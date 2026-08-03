#include "adapters/g1_motion_adapter.hpp"
#include "gmr/geometry_ground.hpp"
#include "gmr/gmr_mink.hpp"

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef G1_BUMI3_REPO_ROOT
#define G1_BUMI3_REPO_ROOT "."
#endif

namespace {

nlohmann::json readJson(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open " + path);
    nlohmann::json value;
    input >> value;
    return value;
}

struct ModelDeleter { void operator()(mjModel* value) const { mj_deleteModel(value); } };

}  // namespace

int main() {
    try {
        const std::string root = G1_BUMI3_REPO_ROOT;
        const std::string source_xml =
            root + "/assets/unitree_g1/g1_mocap_29dof.xml";
        const std::string target_xml = root + "/assets/bumi3/mjcf/bumi3.xml";
        const std::string config_path =
            root + "/config/ik_configs/g1_to_bumi3.json";
        const std::string reference_path =
            root + "/config/ik_configs/smplx_to_bumi3.json";
        const nlohmann::json config = readJson(config_path);
        const nlohmann::json reference = readJson(reference_path);
        if (config.at("source").at("robot") != "unitree_g1" ||
            config.at("target").at("robot") != "bumi3")
            throw std::runtime_error("source/target metadata mismatch");
        if (config.at("root").at("source") != "pelvis" ||
            config.at("root").at("target") != "base_link" ||
            !config.at("root").at("ground_alignment").get<bool>() ||
            config.at("root").at("height_offset").get<double>() != 0.0)
            throw std::runtime_error("root policy mismatch");
        if (config.at("body_mapping").size() != 12)
            throw std::runtime_error("body mapping must contain 12 pairs");

        char error[1024] = {};
        std::unique_ptr<mjModel, ModelDeleter> source_model(
            mj_loadXML(source_xml.c_str(), nullptr, error, sizeof(error)));
        if (!source_model) throw std::runtime_error(error);
        std::unique_ptr<mjModel, ModelDeleter> target_model(
            mj_loadXML(target_xml.c_str(), nullptr, error, sizeof(error)));
        if (!target_model) throw std::runtime_error(error);
        for (const auto& [source, target] : config.at("body_mapping").items()) {
            const std::string target_name = target.get<std::string>();
            if (mj_name2id(source_model.get(), mjOBJ_BODY, source.c_str()) < 0 ||
                mj_name2id(target_model.get(), mjOBJ_BODY,
                           target_name.c_str()) < 0)
                throw std::runtime_error("mapping references missing body: " + source);
            for (const char* table : {"ik_match_table1", "ik_match_table2"}) {
                const auto& entry = config.at(table).at(target_name);
                const auto& weight = reference.at(table).at(target_name);
                if (entry.at(0) != source || entry.at(1) != weight.at(1) ||
                    entry.at(2) != weight.at(2))
                    throw std::runtime_error("generated weights/mapping mismatch: " + source);
            }
        }

        gmr::G1MotionFrame frame;
        frame.root_position = Eigen::Vector3d(0.0, 0.0, 0.793);
        frame.root_rotation = Eigen::Quaterniond::Identity();
        frame.joint_positions.assign(29, 0.0);
        gmr::G1MotionAdapter adapter(source_xml);
        const gmr::BodyMap bodies = adapter.to_body_map(frame);
        gmr_mink::GMR solver(target_xml, config_path, 1.0, 1.0, false);
        Eigen::VectorXd qpos;
        for (int iteration = 0; iteration < 100; ++iteration)
            qpos = solver.retarget(bodies, false);
        gmr::TargetGroundAligner aligner(
            target_xml,
            config.at("root").at("target_foot_bodies")
                .get<std::vector<std::string>>());
        aligner.align(qpos, 0.0);
        if (qpos.size() != 28 || !qpos.allFinite())
            throw std::runtime_error("neutral retarget output invalid");
        const double sole = aligner.lowest(qpos);
        if (std::abs(sole) > 1e-9)
            throw std::runtime_error("model-derived ground alignment failed: " +
                                     std::to_string(sole));
        std::cout << "g1_to_bumi3_config_test: PASS mappings=12 sole_z="
                  << sole << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "g1_to_bumi3_config_test: FAIL: " << error.what() << '\n';
        return 1;
    }
}
