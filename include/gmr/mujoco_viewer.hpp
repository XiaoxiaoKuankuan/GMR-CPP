#pragma once
/**
 * mujoco_viewer.hpp — Passive MuJoCo viewer
 *
 * Loads a MuJoCo XML, renders qpos each frame.
 * Camera follows pelvis when no mouse button is held.
 * Mouse interaction: left=pan, right=rotate, scroll=zoom.
 */

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <Eigen/Dense>
#include "gmr/body_map.hpp"
#include <array>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <atomic>
#include <map>
#include <vector>

extern std::atomic<bool> g_stop;  // defined in main

namespace gmr {

class MujocoViewer {
public:
    explicit MujocoViewer(const std::string& xml_path,
                          int render_width = 640,
                          int render_height = 480,
                          const std::string& follow_body = "pelvis") {
        render_width_ = std::max(160, render_width);
        render_height_ = std::max(120, render_height);

        char err[1000] = {};
        model_ = mj_loadXML(xml_path.c_str(), nullptr, err, sizeof(err));
        if (!model_) throw std::runtime_error("[MujocoViewer] " + std::string(err));
        data_ = mj_makeData(model_);

        if (!glfwInit()) throw std::runtime_error("[GLFW] init failed");
        glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
#ifdef GLFW_SCALE_TO_MONITOR
        glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_FALSE);
#endif
#ifdef GLFW_COCOA_RETINA_FRAMEBUFFER
        glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GLFW_FALSE);
#endif
        window_ = glfwCreateWindow(render_width_, render_height_,
                                   "Mocap Viewer", nullptr, nullptr);
        if (!window_) throw std::runtime_error("[GLFW] window creation failed");
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(0);

        // Store pointer for static callbacks
        glfwSetWindowUserPointer(window_, this);
        glfwSetMouseButtonCallback(window_, cbMouseButton);
        glfwSetCursorPosCallback(window_,   cbMouseMove);
        glfwSetScrollCallback(window_,      cbScroll);

        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);
        mjv_makeScene(model_, &scn_, 2000);
        mjr_makeContext(model_, &con_, mjFONTSCALE_150);

        cam_.distance  = 3.0;
        cam_.azimuth   = 180.0;
        cam_.elevation = -20.0;
        cam_.type      = mjCAMERA_FREE;

        follow_body_id_ = mj_name2id(model_, mjOBJ_BODY, follow_body.c_str());
    }

    ~MujocoViewer() {
        mjr_freeContext(&con_);
        mjv_freeScene(&scn_);
        if (data_)   mj_deleteData(data_);
        if (model_)  mj_deleteModel(model_);
        if (window_) glfwDestroyWindow(window_);
        glfwTerminate();
    }

    // Render one frame. Returns false if window was closed.
    bool render(const Eigen::VectorXd& qpos,
                const BodyMap* raw_targets = nullptr,
                const BodyMap* scaled_targets = nullptr) {
        if (!window_ || glfwWindowShouldClose(window_)) {
            g_stop = true; return false;
        }

        int n = std::min((int)qpos.size(), model_->nq);
        for (int i = 0; i < n; ++i) data_->qpos[i] = qpos[i];
        mj_forward(model_, data_);

        // Camera follows the requested body when not dragging.
        if (follow_body_id_ >= 0 && !mouse_left_ && !mouse_right_ && !mouse_middle_) {
            mjtNum* xpos = data_->xpos + 3 * follow_body_id_;
            cam_.lookat[0] = xpos[0];
            cam_.lookat[1] = xpos[1];
            cam_.lookat[2] = xpos[2];
        }

        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(window_, &fbw, &fbh);
        mjrRect vp{0, 0, std::min(fbw, render_width_),
                   std::min(fbh, render_height_)};
        vp.left = std::max(0, (fbw - vp.width) / 2);
        vp.bottom = std::max(0, (fbh - vp.height) / 2);
        glClearColor(0.f, 0.f, 0.f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
        if (raw_targets && scaled_targets)
            appendTargetOverlay(*raw_targets, *scaled_targets);
        mjr_render(vp, &scn_, &con_);
        glfwSwapBuffers(window_);
        glfwPollEvents();
        return true;
    }

    bool shouldClose() const {
        return window_ && glfwWindowShouldClose(window_);
    }

private:
    mjModel*    model_  = nullptr;
    mjData*     data_   = nullptr;
    GLFWwindow* window_ = nullptr;
    mjvCamera   cam_;
    mjvOption   opt_;
    mjvScene    scn_;
    mjrContext  con_;
    int         follow_body_id_ = -1;
    int         render_width_ = 640;
    int         render_height_ = 480;

    bool   mouse_left_   = false;
    bool   mouse_right_  = false;
    bool   mouse_middle_ = false;
    double mouse_x_      = 0.0;
    double mouse_y_      = 0.0;

    static const std::vector<std::pair<std::string, std::string>>& humanEdges() {
        static const std::vector<std::pair<std::string, std::string>> edges = {
            {"Pelvis", "Chest"},
            {"Pelvis", "Left_UpperLeg"},
            {"Left_UpperLeg", "Left_LowerLeg"},
            {"Left_LowerLeg", "Left_Foot"},
            {"Pelvis", "Right_UpperLeg"},
            {"Right_UpperLeg", "Right_LowerLeg"},
            {"Right_LowerLeg", "Right_Foot"},
            {"Chest", "Left_UpperArm"},
            {"Left_UpperArm", "Left_Forearm"},
            {"Left_Forearm", "Left_Hand"},
            {"Chest", "Right_UpperArm"},
            {"Right_UpperArm", "Right_Forearm"},
            {"Right_Forearm", "Right_Hand"},
        };
        return edges;
    }

    static const std::vector<std::pair<std::string, std::string>>& smplEdges() {
        static const std::vector<std::pair<std::string, std::string>> edges = {
            {"SMPL_Pelvis", "SMPL_Chest"},
            {"SMPL_Pelvis", "SMPL_LeftHip"},
            {"SMPL_LeftHip", "SMPL_LeftKnee"},
            {"SMPL_LeftKnee", "SMPL_LeftAnkle"},
            {"SMPL_Pelvis", "SMPL_RightHip"},
            {"SMPL_RightHip", "SMPL_RightKnee"},
            {"SMPL_RightKnee", "SMPL_RightAnkle"},
            {"SMPL_Chest", "SMPL_LeftShoulder"},
            {"SMPL_LeftShoulder", "SMPL_LeftElbow"},
            {"SMPL_LeftElbow", "SMPL_LeftWrist"},
            {"SMPL_Chest", "SMPL_RightShoulder"},
            {"SMPL_RightShoulder", "SMPL_RightElbow"},
            {"SMPL_RightElbow", "SMPL_RightWrist"},
        };
        return edges;
    }

    static const std::vector<std::pair<std::string, std::string>>& smplxEdges() {
        static const std::vector<std::pair<std::string, std::string>> edges = {
            {"pelvis", "spine3"},
            {"pelvis", "left_hip"},
            {"left_hip", "left_knee"},
            {"left_knee", "left_foot"},
            {"pelvis", "right_hip"},
            {"right_hip", "right_knee"},
            {"right_knee", "right_foot"},
            {"spine3", "left_shoulder"},
            {"left_shoulder", "left_elbow"},
            {"left_elbow", "left_wrist"},
            {"spine3", "right_shoulder"},
            {"right_shoulder", "right_elbow"},
            {"right_elbow", "right_wrist"},
        };
        return edges;
    }

    static const std::map<std::string, std::string>& robotBodies() {
        static const std::map<std::string, std::string> bodies = {
            {"Pelvis", "base_link"},
            {"Chest", "waist_roll_link"},
            {"Left_UpperLeg", "l_leg_hip_pitch_link"},
            {"Left_LowerLeg", "l_leg_knee_link"},
            {"Left_Foot", "l_leg_ankle_roll_link"},
            {"Right_UpperLeg", "r_leg_hip_pitch_link"},
            {"Right_LowerLeg", "r_leg_knee_link"},
            {"Right_Foot", "r_leg_ankle_roll_link"},
            {"Left_UpperArm", "l_arm_shoulder_roll_link"},
            {"Left_Forearm", "l_arm_elbow_pitch_link"},
            {"Left_Hand", "l_arm_elbow_yaw_link"},
            {"Right_UpperArm", "r_arm_shoulder_roll_link"},
            {"Right_Forearm", "r_arm_elbow_pitch_link"},
            {"Right_Hand", "r_arm_elbow_yaw_link"},
        };
        return bodies;
    }

    static const std::map<std::string, std::string>& smplRobotBodies() {
        static const std::map<std::string, std::string> bodies = {
            {"SMPL_Pelvis", "base_link"},
            {"SMPL_Chest", "waist_roll_link"},
            {"SMPL_LeftHip", "l_leg_hip_pitch_link"},
            {"SMPL_LeftKnee", "l_leg_knee_link"},
            {"SMPL_LeftAnkle", "l_leg_ankle_roll_link"},
            {"SMPL_RightHip", "r_leg_hip_pitch_link"},
            {"SMPL_RightKnee", "r_leg_knee_link"},
            {"SMPL_RightAnkle", "r_leg_ankle_roll_link"},
            {"SMPL_LeftShoulder", "l_arm_shoulder_roll_link"},
            {"SMPL_LeftElbow", "l_arm_elbow_pitch_link"},
            {"SMPL_LeftWrist", "l_arm_elbow_yaw_link"},
            {"SMPL_RightShoulder", "r_arm_shoulder_roll_link"},
            {"SMPL_RightElbow", "r_arm_elbow_pitch_link"},
            {"SMPL_RightWrist", "r_arm_elbow_yaw_link"},
        };
        return bodies;
    }

    static const std::map<std::string, std::string>& smplxRobotBodiesE1() {
        static const std::map<std::string, std::string> bodies = {
            {"pelvis", "base_link"},
            {"spine3", "waist_roll_link"},
            {"left_hip", "l_leg_hip_pitch_link"},
            {"left_knee", "l_leg_knee_link"},
            {"left_foot", "l_leg_ankle_roll_link"},
            {"right_hip", "r_leg_hip_pitch_link"},
            {"right_knee", "r_leg_knee_link"},
            {"right_foot", "r_leg_ankle_roll_link"},
            {"left_shoulder", "l_arm_shoulder_roll_link"},
            {"left_elbow", "l_arm_elbow_pitch_link"},
            {"left_wrist", "l_arm_elbow_yaw_link"},
            {"right_shoulder", "r_arm_shoulder_roll_link"},
            {"right_elbow", "r_arm_elbow_pitch_link"},
            {"right_wrist", "r_arm_elbow_yaw_link"},
        };
        return bodies;
    }

    static const std::map<std::string, std::string>& smplxRobotBodiesG1() {
        static const std::map<std::string, std::string> bodies = {
            {"pelvis", "pelvis"},
            {"spine3", "torso_link"},
            {"left_hip", "left_hip_roll_link"},
            {"left_knee", "left_knee_link"},
            {"left_foot", "left_toe_link"},
            {"right_hip", "right_hip_roll_link"},
            {"right_knee", "right_knee_link"},
            {"right_foot", "right_toe_link"},
            {"left_shoulder", "left_shoulder_yaw_link"},
            {"left_elbow", "left_elbow_link"},
            {"left_wrist", "left_wrist_yaw_link"},
            {"right_shoulder", "right_shoulder_yaw_link"},
            {"right_elbow", "right_elbow_link"},
            {"right_wrist", "right_wrist_yaw_link"},
        };
        return bodies;
    }

    enum class TargetSchema { Legacy, SmplDirect, SmplxReference };

    static TargetSchema targetSchema(const BodyMap& poses) {
        if (poses.count("SMPL_Pelvis")) return TargetSchema::SmplDirect;
        if (poses.count("pelvis") && poses.count("spine3"))
            return TargetSchema::SmplxReference;
        return TargetSchema::Legacy;
    }

    void appendSphere(const Eigen::Vector3d& position,
                      const std::array<float, 4>& color,
                      double radius) {
        if (scn_.ngeom >= scn_.maxgeom || !validOverlayPosition(position)) return;
        mjtNum size[3] = {radius, radius, radius};
        mjtNum pos[3] = {position.x(), position.y(), position.z()};
        mjvGeom* geom = &scn_.geoms[scn_.ngeom++];
        mjv_initGeom(geom, mjGEOM_SPHERE, size, pos, nullptr, color.data());
        geom->category = mjCAT_DECOR;
    }

    void appendLine(const Eigen::Vector3d& first,
                    const Eigen::Vector3d& second,
                    const std::array<float, 4>& color,
                    double width) {
        if (scn_.ngeom >= scn_.maxgeom ||
            !validOverlayPosition(first) || !validOverlayPosition(second) ||
            (second - first).squaredNorm() < 1e-12) {
            return;
        }

        // MuJoCo requires mjv_initGeom before mjv_makeConnector.  Calling the
        // latter on an unused scene slot leaves fields such as objtype/segid
        // uninitialized and can crash mjr_render depending on the frame data.
        mjtNum size[3] = {width, width, width};
        mjtNum pos[3] = {0.0, 0.0, 0.0};
        mjvGeom* geom = &scn_.geoms[scn_.ngeom++];
        mjv_initGeom(
            geom, mjGEOM_CAPSULE, size, pos, nullptr, color.data());
        mjv_makeConnector(
            geom, mjGEOM_CAPSULE, width,
            first.x(), first.y(), first.z(),
            second.x(), second.y(), second.z());
        geom->category = mjCAT_DECOR;
    }

    static bool validOverlayPosition(const Eigen::Vector3d& position) {
        constexpr double kMaxOverlayCoordinate = 100.0;
        return position.allFinite() &&
               position.cwiseAbs().maxCoeff() <= kMaxOverlayCoordinate;
    }

    void appendSkeleton(const BodyMap& poses,
                        const std::array<float, 4>& color,
                        double radius) {
        for (const auto& [name, pose] : poses) {
            (void)name;
            appendSphere(pose.position, color, radius);
        }
        const auto schema = targetSchema(poses);
        const auto& edges = schema == TargetSchema::SmplDirect ? smplEdges() :
                            schema == TargetSchema::SmplxReference ? smplxEdges() :
                            humanEdges();
        for (const auto& [first, second] : edges) {
            auto a = poses.find(first);
            auto b = poses.find(second);
            if (a != poses.end() && b != poses.end())
                appendLine(a->second.position, b->second.position, color, radius * 0.45);
        }
    }

    BodyMap currentRobotBodies(TargetSchema schema) const {
        BodyMap poses;
        const std::map<std::string, std::string>* bodies = &robotBodies();
        if (schema == TargetSchema::SmplDirect) {
            bodies = &smplRobotBodies();
        } else if (schema == TargetSchema::SmplxReference) {
            const bool is_e1 = mj_name2id(model_, mjOBJ_BODY, "base_link") >= 0;
            bodies = is_e1 ? &smplxRobotBodiesE1() : &smplxRobotBodiesG1();
        }
        for (const auto& [human_name, robot_name] : *bodies) {
            int body_id = mj_name2id(model_, mjOBJ_BODY, robot_name.c_str());
            if (body_id < 0) continue;
            const mjtNum* position = data_->xpos + 3 * body_id;
            BodyData pose;
            pose.position = Eigen::Vector3d(position[0], position[1], position[2]);
            poses.emplace(human_name, pose);
        }
        return poses;
    }

    void appendTargetOverlay(const BodyMap& raw_targets,
                             const BodyMap& scaled_targets) {
        const std::array<float, 4> blue   = {0.10F, 0.35F, 1.00F, 0.85F};
        const std::array<float, 4> yellow = {1.00F, 0.80F, 0.10F, 0.90F};
        const std::array<float, 4> white  = {1.00F, 1.00F, 1.00F, 0.90F};
        const auto schema = targetSchema(raw_targets);
        appendSkeleton(raw_targets, blue, 0.018);
        appendSkeleton(scaled_targets, yellow, 0.022);
        appendSkeleton(currentRobotBodies(schema), white, 0.014);
    }

    static MujocoViewer* get(GLFWwindow* w) {
        return static_cast<MujocoViewer*>(glfwGetWindowUserPointer(w));
    }

    static void cbMouseButton(GLFWwindow* w, int btn, int act, int) {
        auto* v = get(w);
        if (!v) return;
        if (btn == GLFW_MOUSE_BUTTON_LEFT)   v->mouse_left_   = (act == GLFW_PRESS);
        if (btn == GLFW_MOUSE_BUTTON_RIGHT)  v->mouse_right_  = (act == GLFW_PRESS);
        if (btn == GLFW_MOUSE_BUTTON_MIDDLE) v->mouse_middle_ = (act == GLFW_PRESS);
        glfwGetCursorPos(w, &v->mouse_x_, &v->mouse_y_);
    }

    static void cbMouseMove(GLFWwindow* w, double xpos, double ypos) {
        auto* v = get(w);
        if (!v) return;
        double dx = xpos - v->mouse_x_;
        double dy = ypos - v->mouse_y_;
        v->mouse_x_ = xpos; v->mouse_y_ = ypos;
        if (!v->mouse_left_ && !v->mouse_right_ && !v->mouse_middle_) return;
        int ww, wh;
        glfwGetWindowSize(w, &ww, &wh);
        mjtMouse action = v->mouse_right_ ? mjMOUSE_ROTATE_V :
                          v->mouse_left_  ? mjMOUSE_MOVE_H   : mjMOUSE_ZOOM;
        mjv_moveCamera(v->model_, action, dx/wh, dy/wh, &v->scn_, &v->cam_);
    }

    static void cbScroll(GLFWwindow* w, double, double dy) {
        auto* v = get(w);
        if (!v) return;
        mjv_moveCamera(v->model_, mjMOUSE_ZOOM, 0, -0.05*dy, &v->scn_, &v->cam_);
    }
};

} // namespace gmr
