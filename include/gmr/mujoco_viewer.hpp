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
#include <string>
#include <stdexcept>
#include <algorithm>
#include <atomic>

extern std::atomic<bool> g_stop;  // defined in main

namespace gmr {

class MujocoViewer {
public:
    explicit MujocoViewer(const std::string& xml_path,
                          int render_width = 640,
                          int render_height = 480) {
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

        pelvis_id_ = mj_name2id(model_, mjOBJ_BODY, "pelvis");
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
    bool render(const Eigen::VectorXd& qpos) {
        if (!window_ || glfwWindowShouldClose(window_)) {
            g_stop = true; return false;
        }

        int n = std::min((int)qpos.size(), model_->nq);
        for (int i = 0; i < n; ++i) data_->qpos[i] = qpos[i];
        mj_forward(model_, data_);

        // Camera follows pelvis when not dragging
        if (pelvis_id_ >= 0 && !mouse_left_ && !mouse_right_ && !mouse_middle_) {
            mjtNum* xpos = data_->xpos + 3 * pelvis_id_;
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
    int         pelvis_id_ = -1;
    int         render_width_ = 640;
    int         render_height_ = 480;

    bool   mouse_left_   = false;
    bool   mouse_right_  = false;
    bool   mouse_middle_ = false;
    double mouse_x_      = 0.0;
    double mouse_y_      = 0.0;

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
