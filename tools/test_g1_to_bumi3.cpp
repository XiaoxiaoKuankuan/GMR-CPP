#include "adapters/g1_motion_adapter.hpp"
#include "gmr/geometry_ground.hpp"
#include "gmr/gmr_mink.hpp"
#include "gmr/npy_io.hpp"
#include "readers/g1_motion_reader.hpp"

#include <mujoco/mujoco.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Options {
    std::string input;
    std::string config;
    std::string source_xml;
    std::string target_xml;
    std::string output = "bumi3_motion.npy";
    double fps = 30.0;
};

Options parseArgs(int argc, char** argv) {
    Options options;
    const std::filesystem::path executable =
        std::filesystem::weakly_canonical(argv[0]);
    const std::filesystem::path root = executable.parent_path().parent_path();
    options.source_xml = (root / "assets/unitree_g1/g1_mocap_29dof.xml").string();
    options.target_xml = (root / "assets/bumi3/mjcf/bumi3.xml").string();
    for (int index = 1; index < argc; ++index) {
        const std::string arg = argv[index];
        auto next = [&]() {
            if (++index >= argc) throw std::runtime_error("missing value for " + arg);
            return std::string(argv[index]);
        };
        if (arg == "--input") options.input = next();
        else if (arg == "--config") options.config = next();
        else if (arg == "--source-xml") options.source_xml = next();
        else if (arg == "--target-xml") options.target_xml = next();
        else if (arg == "--output") options.output = next();
        else if (arg == "--fps") options.fps = std::stod(next());
        else if (arg == "--help" || arg == "-h") {
            std::cout
                << "Usage: " << argv[0]
                << " --input walk.npy --config config/ik_configs/g1_to_bumi3.json\n"
                   "       [--fps 30] [--output bumi3_motion.npy]"
                   " [--source-xml g1.xml] [--target-xml bumi3.xml]\n";
            std::exit(0);
        } else throw std::runtime_error("unknown argument: " + arg);
    }
    if (options.input.empty() || options.config.empty())
        throw std::runtime_error("--input and --config are required");
    return options;
}

nlohmann::json readJson(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open JSON: " + path);
    nlohmann::json value;
    input >> value;
    return value;
}

struct ModelDeleter { void operator()(mjModel* value) const { mj_deleteModel(value); } };
struct DataDeleter { void operator()(mjData* value) const { mj_deleteData(value); } };

}  // namespace

int main(int argc, char** argv) {
    try {
        const Options options = parseArgs(argc, argv);
        const nlohmann::json config = readJson(options.config);
        if (config.at("source").at("robot") != "unitree_g1" ||
            config.at("target").at("robot") != "bumi3")
            throw std::runtime_error("config source/target must be unitree_g1/bumi3");
        const auto& root = config.at("root");
        const bool ground_alignment = root.at("ground_alignment").get<bool>();
        const double height_offset = root.at("height_offset").get<double>();
        const std::vector<std::string> target_feet =
            root.at("target_foot_bodies").get<std::vector<std::string>>();

        gmr::G1MotionReader reader(options.fps);
        if (!reader.load_npy(options.input))
            throw std::runtime_error(reader.last_error());
        gmr::G1MotionAdapter adapter(options.source_xml);
        gmr_mink::GMR solver(options.target_xml, options.config, 1.0, 1.0, false);
        gmr::TargetGroundAligner ground(options.target_xml, target_feet);

        char error[1024] = {};
        std::unique_ptr<mjModel, ModelDeleter> target_model(
            mj_loadXML(options.target_xml.c_str(), nullptr, error, sizeof(error)));
        if (!target_model) throw std::runtime_error(error);
        std::unique_ptr<mjData, DataDeleter> target_data(mj_makeData(target_model.get()));
        if (!target_data) throw std::runtime_error("mj_makeData failed");
        const int left_foot = mj_name2id(
            target_model.get(), mjOBJ_BODY, target_feet.at(0).c_str());
        const int right_foot = mj_name2id(
            target_model.get(), mjOBJ_BODY, target_feet.at(1).c_str());
        if (left_foot < 0 || right_foot < 0)
            throw std::runtime_error("target foot body missing");

        std::vector<double> motion;
        std::vector<double> roots;
        std::vector<double> feet;
        gmr::G1MotionFrame frame;
        size_t frame_index = 0;
        double minimum_sole = std::numeric_limits<double>::infinity();
        double maximum_sole = -std::numeric_limits<double>::infinity();
        while (reader.get_next_frame(frame)) {
            const gmr::BodyMap source_bodies = adapter.to_body_map(frame);
            Eigen::VectorXd qpos;
            const int iterations = frame_index == 0 ? 100 : 1;
            for (int iteration = 0; iteration < iterations; ++iteration)
                qpos = solver.retarget(source_bodies, false);
            if (ground_alignment) ground.align(qpos, height_offset);
            if (!qpos.allFinite())
                throw std::runtime_error("non-finite target qpos at frame " +
                                         std::to_string(frame_index));
            for (int index = 0; index < qpos.size(); ++index)
                motion.push_back(qpos[index]);
            for (int index = 0; index < 7; ++index) roots.push_back(qpos[index]);

            for (int index = 0; index < target_model->nq; ++index)
                target_data->qpos[index] = qpos[index];
            mj_forward(target_model.get(), target_data.get());
            for (const int body : {left_foot, right_foot}) {
                feet.push_back(target_data->xpos[3 * body]);
                feet.push_back(target_data->xpos[3 * body + 1]);
                feet.push_back(target_data->xpos[3 * body + 2]);
            }
            const double sole = ground.lowest(qpos);
            minimum_sole = std::min(minimum_sole, sole);
            maximum_sole = std::max(maximum_sole, sole);
            ++frame_index;
        }
        if (frame_index == 0) throw std::runtime_error("input motion is empty");

        const std::filesystem::path motion_path = options.output;
        const std::filesystem::path directory = motion_path.has_parent_path()
            ? motion_path.parent_path() : std::filesystem::path(".");
        std::filesystem::create_directories(directory);
        const std::string stem = motion_path.stem().string();
        const std::filesystem::path root_path = directory / (stem + "_root.npy");
        const std::filesystem::path foot_path = directory / (stem + "_feet.npy");
        const std::filesystem::path replay_path = directory / (stem + "_replay.json");
        gmr::npy::save2D(motion_path.string(), motion, frame_index,
                         static_cast<size_t>(target_model->nq));
        gmr::npy::save2D(root_path.string(), roots, frame_index, 7);
        gmr::npy::save2D(foot_path.string(), feet, frame_index, 6);

        nlohmann::json replay = {
            {"format", "gmr_mujoco_qpos_replay_v1"},
            {"robot", "bumi3"}, {"xml", options.target_xml},
            {"qpos", std::filesystem::absolute(motion_path).string()},
            {"root_trajectory", std::filesystem::absolute(root_path).string()},
            {"foot_trajectory", std::filesystem::absolute(foot_path).string()},
            {"fps", options.fps}, {"frames", frame_index},
            {"nq", target_model->nq},
            {"ground_alignment", ground_alignment},
            {"height_offset", height_offset},
        };
        std::ofstream replay_file(replay_path);
        replay_file << std::setw(2) << replay << '\n';
        if (!replay_file) throw std::runtime_error("failed writing replay manifest");

        std::cout << "[offline] input=" << options.input << " frames=" << frame_index
                  << " fps=" << options.fps << "\n"
                  << "[offline] qpos=" << motion_path << " shape=(" << frame_index
                  << ',' << target_model->nq << ")\n"
                  << "[offline] root=" << root_path << " shape=(" << frame_index
                  << ",7)\n"
                  << "[offline] feet=" << foot_path << " shape=(" << frame_index
                  << ",6)\n"
                  << "[offline] replay=" << replay_path << "\n"
                  << "[offline] sole_z_range=[" << minimum_sole << ','
                  << maximum_sole << "] height_offset=" << height_offset << "\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test_g1_to_bumi3: " << error.what() << '\n';
        return 1;
    }
}
