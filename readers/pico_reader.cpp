/**
 * pico_reader.cpp
 */

#include "pico_reader.hpp"
#include <stdexcept>

namespace gmr {

using json = nlohmann::json;

const std::vector<std::string> PicoReader::kBoneNames = {
    "Pelvis","Left_Hip","Right_Hip","Spine1","Left_Knee","Right_Knee",
    "Spine2","Left_Ankle","Right_Ankle","Spine3","Left_Foot","Right_Foot",
    "Neck","Left_Collar","Right_Collar","Head","Left_Shoulder","Right_Shoulder",
    "Left_Elbow","Right_Elbow","Left_Wrist","Right_Wrist","Left_Hand","Right_Hand",
};

const std::map<std::string,std::string> PicoReader::kDefaultMap = {
    {"Pelvis","Pelvis"},{"Left_Hip","Left_Hip"},{"Right_Hip","Right_Hip"},
    {"Spine1","Spine3"},{"Spine3","Spine3"},
    {"Left_Knee","Left_Knee"},{"Right_Knee","Right_Knee"},
    {"Left_Ankle","Left_Foot"},{"Right_Ankle","Right_Foot"},
    {"Left_Shoulder","Left_Shoulder"},{"Right_Shoulder","Right_Shoulder"},
    {"Left_Elbow","Left_Elbow"},{"Right_Elbow","Right_Elbow"},
    {"Left_Wrist","Left_Wrist"},{"Right_Wrist","Right_Wrist"},
};

PicoReader::PicoReader(FrameQueue& queue, Config cfg)
    : BaseReader(queue), cfg_(std::move(cfg))
{
    if (cfg_.bone_name_map.empty()) cfg_.bone_name_map = kDefaultMap;
}

PicoReader::~PicoReader() { disconnect(); }

void PicoReader::connect() {
    if (PXREAInit(this, pxreaCallback, PXREAFullMask) != 0)
        throw std::runtime_error("[PicoReader] PXREAInit failed");
    auto t0 = std::chrono::steady_clock::now();
    while (!isBodyDataAvailable()) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count() > 10.0)
            throw std::runtime_error("[PicoReader] Timed out waiting for body data.");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    connected_ = true;
    fetch_running_ = true;
    fetch_thread_ = std::thread(&PicoReader::fetchLoop, this);
    std::cout << "[PicoReader] Connected. Body tracking available.\n";
}

void PicoReader::disconnect() {
    fetch_running_ = false;
    if (fetch_thread_.joinable()) fetch_thread_.join();
    if (connected_) { PXREADeinit(); connected_ = false; }
}

void PicoReader::fetchLoop() {
    // Data is pushed directly from callback — fetchLoop not needed
    while (fetch_running_)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void PicoReader::pxreaCallback(
    void* ctx, PXREAClientCallbackType type, int, void* user_data)
{
    if (type != PXREADeviceStateJson || !ctx || !user_data) return;
    auto* self = static_cast<PicoReader*>(ctx);
    auto& dsj  = *static_cast<PXREADevStateJson*>(user_data);
    self->onStateJson(dsj.stateJson);
}

void PicoReader::onStateJson(const char* json_str) {
    static int cb_count = 0;
    static auto cb_t0 = std::chrono::steady_clock::now();
    if (++cb_count % 100 == 0) {
        double el = std::chrono::duration<double>(std::chrono::steady_clock::now()-cb_t0).count();
        std::printf("[PicoCallback] freq=%.1fHz\n", cb_count/el);
    }
    try {
        json data = json::parse(json_str);
        if (!data.contains("value")) return;
        json value = json::parse(data["value"].get<std::string>());
        if (!value.contains("Body")) return;
        auto& body = value["Body"];

        // Use monotonic clock recorded at callback time — mirrors Python's behavior
        // where xrt.get_time_stamp_ns() is read at 80Hz poll time
        int64_t stamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        if (!body.contains("joints") || !body["joints"].is_array()) return;
        auto& joints_json = body["joints"];
        int count = std::min(int(joints_json.size()), 24);

        std::array<std::array<double,7>,24> joints{};
        for (int i = 0; i < count; ++i) {
            auto& j = joints_json[i];
            if (j.contains("p"))
                joints[i] = parsePoseStr(j["p"].get<std::string>());
        }

        {
            std::lock_guard<std::mutex> lk(body_mtx_);
            body_joints_   = joints;
            body_stamp_ns_ = stamp_ns;
            body_available_ = true;
        }

        // Push directly from callback for minimum latency
        RawFrame raw;
        raw.stamp_ns = stamp_ns ? stamp_ns :
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        raw.frame_number = int(raw.stamp_ns / 1000000);
        // TEMP: print raw Pelvis data once
        static bool printed = false;
        if (!printed) {
            printed = true;
            std::printf("[RAW] Pelvis:    x=%.4f y=%.4f z=%.4f qx=%.4f qy=%.4f qz=%.4f qw=%.4f\n",
                joints[0][0], joints[0][1], joints[0][2], joints[0][3], joints[0][4], joints[0][5], joints[0][6]);
            std::printf("[RAW] Left_Hip:  x=%.4f y=%.4f z=%.4f qx=%.4f qy=%.4f qz=%.4f qw=%.4f\n",
                joints[1][0], joints[1][1], joints[1][2], joints[1][3], joints[1][4], joints[1][5], joints[1][6]);
            std::printf("[RAW] Right_Hip: x=%.4f y=%.4f z=%.4f qx=%.4f qy=%.4f qz=%.4f qw=%.4f\n",
                joints[2][0], joints[2][1], joints[2][2], joints[2][3], joints[2][4], joints[2][5], joints[2][6]);
        }
        for (size_t i = 0; i < kBoneNames.size(); ++i) {
            auto it = cfg_.bone_name_map.find(kBoneNames[i]);
            if (it == cfg_.bone_name_map.end()) continue;
            BodyData bd;
            transformUnityToRH(joints[i], bd.position, bd.rot_wxyz);
            raw.body_data[it->second] = bd;
        }

        // 打印放在循环外面，所有骨骼都已处理完
        static bool printed2 = false;
        if (!printed2) {
            printed2 = true;
            for (auto& name : {"Pelvis", "Left_Hip", "Right_Hip"}) {
                auto it2 = raw.body_data.find(name);
                if (it2 != raw.body_data.end())
                    std::printf("[CPP_TRANSFORMED] %s: pos=[%.4f,%.4f,%.4f] rot=[%.4f,%.4f,%.4f,%.4f]\n",
                        name,
                        it2->second.position[0], it2->second.position[1], it2->second.position[2],
                        it2->second.rot_wxyz[0], it2->second.rot_wxyz[1], it2->second.rot_wxyz[2], it2->second.rot_wxyz[3]);
            }
        }

        if (!raw.body_data.empty()) queue_.push(std::move(raw));

    } catch (const json::exception& e) {
        std::cerr << "[PicoReader] JSON error: " << e.what() << "\n";
    }
}

std::array<double,7> PicoReader::parsePoseStr(const std::string& s) {
    std::array<double,7> r{};
    std::stringstream ss(s);
    std::string tok;
    int i = 0;
    while (std::getline(ss, tok, ',') && i < 7) r[i++] = std::stod(tok);
    return r;
}

void PicoReader::transformUnityToRH(
    const std::array<double,7>& p,
    Eigen::Vector3d& pos_out,
    Eigen::Vector4d& quat_wxyz_out)
{
    pos_out[0] =  p[0];
    pos_out[1] = -p[2];
    pos_out[2] =  p[1];
    double bw=p[6],bx=p[3],by=p[4],bz=p[5];
    double aw=0.70710678, ax=0.70710678, ay=0.0, az=0.0;
    quat_wxyz_out[0]=aw*bw-ax*bx-ay*by-az*bz;
    quat_wxyz_out[1]=aw*bx+ax*bw+ay*bz-az*by;
    quat_wxyz_out[2]=aw*by-ax*bz+ay*bw+az*bx;
    quat_wxyz_out[3]=aw*bz+ax*by-ay*bx+az*bw;
    double n=quat_wxyz_out.norm();
    if (n>1e-12) quat_wxyz_out/=n;
    else quat_wxyz_out=Eigen::Vector4d(1,0,0,0);
}

} // namespace gmr