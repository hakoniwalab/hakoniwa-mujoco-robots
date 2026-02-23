#pragma once

#include <memory>
#include <string>

#include "hako_msgs/pdu_cpptype_conv_GameControllerOperation.hpp"
#include "controller/forklift_controller.hpp"
#include "hakoniwa_mujoco_context.hpp"

class ForkliftPduRuntime {
public:
    ForkliftPduRuntime(const std::string& robot_name, const std::string& fork_robot_name);
    ~ForkliftPduRuntime();

    bool load_pad(HakoCpp_GameControllerOperation& out_pad);
    void publish_state(
        hako::robots::controller::ForkliftController& controller,
        const HakoniwaMujocoContext::ControlState& control_state);

private:
    class Impl;
    std::unique_ptr<Impl> impl_ {};
};
