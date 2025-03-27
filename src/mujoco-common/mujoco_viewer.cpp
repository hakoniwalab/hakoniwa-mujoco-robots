#include "mujoco_viewer.hpp"

#define GL_SILENCE_DEPRECATION  // OpenGL の非推奨警告を無効化

#include <GLFW/glfw3.h>
#include <sstream> // 文字列ストリームで時間表示
#include <iomanip>
#include <iostream>
#include "mujoco_viewer.hpp"
#include <sstream>
#include <iomanip>
#include <thread>

// OpenGL & MuJoCo 変数
static mjvCamera cam;
static mjvOption opt;
static mjvScene scn;
static mjrContext con;
static GLFWwindow* window;

// マウスの状態
static bool mouse_button_left = false;
static bool mouse_button_right = false;
static bool mouse_shift_down = false;
static double last_x, last_y;
static mjModel* mujoco_model = nullptr;  // グローバルにモデルを格納

// マウスクリックのコールバック
static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    mouse_button_left = (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS);   // **右クリックで視点回転**
    mouse_button_right = (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS); // **中央クリックで移動**
    mouse_shift_down = (mods & GLFW_MOD_SHIFT);  // Shiftキーが押されているか
    glfwGetCursorPos(window, &last_x, &last_y);
}



// マウス移動のコールバック
static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
    if (!mouse_button_left && !mouse_button_right) return;

    double dx = xpos - last_x;
    double dy = ypos - last_y;
    last_x = xpos;
    last_y = ypos;

    int mode;
    if (mouse_button_left) {
        mode = mjMOUSE_ROTATE_H;  // **右クリックで視点回転**
    } else if (mouse_shift_down && mouse_button_right) {
        mode = mjMOUSE_ZOOM;      // **Shift + 中央クリックでズーム**
    } else if (mouse_button_right) {
        mode = mjMOUSE_MOVE_V;    // **中央クリックでカメラ移動**
    } else {
        return;
    }

    mjv_moveCamera(mujoco_model, mode, dx / 200.0, dy / 200.0, &scn, &cam);
}


// マウスホイールのコールバック（ズーム）
static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(mujoco_model, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
}


// キーボードコールバック
static void keyboard(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_ESCAPE) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);  // ESCでウィンドウを閉じる
        }
    }
}

// 3Dビューアとシミュレーションを統合
void viewer_thread(mjModel* model, mjData* data, bool& running_flag, std::mutex& mutex) {
    if (!glfwInit()) {
        std::cerr << "[ERROR] GLFW Initialization failed!" << std::endl;
        return;
    }
    mujoco_model = model; 
    window = glfwCreateWindow(800, 600, "MuJoCo Simulation Viewer", NULL, NULL);
    if (!window) {
        std::cerr << "[ERROR] GLFW Window creation failed!" << std::endl;
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetKeyCallback(window, keyboard);

    // **マウス操作を有効化**
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, mouse_move_callback);
    glfwSetScrollCallback(window, scroll_callback);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_makeScene(model, &scn, 2000);
    mjr_makeContext(model, &con, mjFONTSCALE_150);

    mjrRect viewport = {0, 0, 800, 600};
    std::cout << "[INFO] Viewer thread started." << std::endl;
    while (!glfwWindowShouldClose(window)) {
        {
            std::lock_guard<std::mutex> lock(mutex);
            mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            mjr_render(viewport, &scn, &con);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwDestroyWindow(window);
    glfwTerminate();
    std::cout << "[INFO] Viewer thread finished." << std::endl;
}

