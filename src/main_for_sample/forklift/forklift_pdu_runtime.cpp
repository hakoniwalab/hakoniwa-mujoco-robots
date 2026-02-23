#include "forklift_pdu_runtime.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include "hako_asset.h"
#include "hako_asset_pdu.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"
#include "std_msgs/pdu_cpptype_conv_Float64.hpp"
#include "std_msgs/pdu_cpptype_conv_Int32.hpp"
#include "hako_msgs/pdu_cpptype_conv_GameControllerOperation.hpp"

namespace {
bool resolve_pdu_info(
    const std::string& robot_name,
    const std::string& pdu_name,
    int& pdu_size,
    int& channel_id)
{
    std::vector<hako::asset::Robot> robots;
    if (!hako::asset::hako_asset_get_pdus(robots)) {
        return false;
    }
    for (const auto& robot : robots) {
        if (robot.name != robot_name) {
            continue;
        }
        for (const auto& writer : robot.pdu_writers) {
            if (writer.org_name == pdu_name) {
                pdu_size = writer.pdu_size;
                channel_id = writer.channel_id;
                return true;
            }
        }
        for (const auto& reader : robot.pdu_readers) {
            if (reader.org_name == pdu_name) {
                pdu_size = reader.pdu_size;
                channel_id = reader.channel_id;
                return true;
            }
        }
    }
    return false;
}

template <typename CppType, typename Convertor>
class PduChannel {
public:
    PduChannel(const std::string& robot_name, const std::string& pdu_name)
        : robot_name_(robot_name)
    {
        int pdu_size = 0;
        int channel_id = -1;
        if (!resolve_pdu_info(robot_name, pdu_name, pdu_size, channel_id)) {
            throw std::runtime_error("PDU not found: robot=" + robot_name + " pdu=" + pdu_name);
        }
        channel_id_ = channel_id;
        buffer_.resize(static_cast<std::size_t>(pdu_size));
    }

    bool load(CppType& data)
    {
        if (hako_asset_pdu_read(robot_name_.c_str(), channel_id_, buffer_.data(), buffer_.size()) != 0) {
            return false;
        }
        auto* meta = reinterpret_cast<const HakoPduMetaDataType*>(buffer_.data());
        if (HAKO_PDU_METADATA_IS_INVALID(meta)) {
            return false;
        }
        if (hako_get_base_ptr_pdu(static_cast<void*>(buffer_.data())) == nullptr) {
            return false;
        }
        return convertor_.pdu2cpp(buffer_.data(), data);
    }

    bool flush(CppType& data)
    {
        int actual_size = convertor_.cpp2pdu(data, buffer_.data(), static_cast<int>(buffer_.size()));
        if (actual_size <= 0 || actual_size > static_cast<int>(buffer_.size())) {
            return false;
        }
        return hako_asset_pdu_write(
                   robot_name_.c_str(),
                   channel_id_,
                   buffer_.data(),
                   static_cast<std::size_t>(actual_size))
            == 0;
    }

private:
    std::string robot_name_ {};
    int channel_id_ {-1};
    Convertor convertor_ {};
    std::vector<char> buffer_ {};
};
} // namespace

class ForkliftPduRuntime::Impl {
public:
    Impl(const std::string& robot_name, const std::string& fork_robot_name)
        : pad_(robot_name, "hako_cmd_game")
        , forklift_pos_(robot_name, "pos")
        , lift_pos_(robot_name, "height")
        , phase_pos_(robot_name, "phase")
        , forklift_fork_pos_(fork_robot_name, "pos")
    {
    }

    bool load_pad(HakoCpp_GameControllerOperation& out_pad)
    {
        return pad_.load(out_pad);
    }

    void publish_state(
        hako::robots::controller::ForkliftController& controller,
        const HakoniwaMujocoContext::ControlState& control_state)
    {
        HakoCpp_Twist forklift_pos_data {};
        forklift_pos_data.linear.x = controller.getForklift().getPosition().x;
        forklift_pos_data.linear.y = controller.getForklift().getPosition().y;
        forklift_pos_data.linear.z = controller.getForklift().getPosition().z;
        forklift_pos_data.angular.x = controller.getForklift().getEuler().x;
        forklift_pos_data.angular.y = controller.getForklift().getEuler().y;
        forklift_pos_data.angular.z = controller.getForklift().getEuler().z;
        (void)forklift_pos_.flush(forklift_pos_data);

        HakoCpp_Float64 lift_pos_data {};
        lift_pos_data.data = controller.getForklift().getLiftPosition().z;
        (void)lift_pos_.flush(lift_pos_data);

        HakoCpp_Int32 phase_pos_data {};
        phase_pos_data.data = static_cast<int32_t>(control_state.phase);
        (void)phase_pos_.flush(phase_pos_data);

        HakoCpp_Twist fork_pos_data {};
        fork_pos_data.linear.x = controller.getForklift().getLiftWorldPosition().x;
        fork_pos_data.linear.y = controller.getForklift().getLiftWorldPosition().y;
        fork_pos_data.linear.z = controller.getForklift().getLiftWorldPosition().z;
        fork_pos_data.angular.x = controller.getForklift().getLiftEuler().x;
        fork_pos_data.angular.y = controller.getForklift().getLiftEuler().y;
        fork_pos_data.angular.z = controller.getForklift().getLiftEuler().z;
        (void)forklift_fork_pos_.flush(fork_pos_data);
    }

private:
    PduChannel<HakoCpp_GameControllerOperation, hako::pdu::msgs::hako_msgs::GameControllerOperation> pad_;
    PduChannel<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist> forklift_pos_;
    PduChannel<HakoCpp_Float64, hako::pdu::msgs::std_msgs::Float64> lift_pos_;
    PduChannel<HakoCpp_Int32, hako::pdu::msgs::std_msgs::Int32> phase_pos_;
    PduChannel<HakoCpp_Twist, hako::pdu::msgs::geometry_msgs::Twist> forklift_fork_pos_;
};

ForkliftPduRuntime::ForkliftPduRuntime(
    const std::string& robot_name,
    const std::string& fork_robot_name)
    : impl_(std::make_unique<Impl>(robot_name, fork_robot_name))
{
}

ForkliftPduRuntime::~ForkliftPduRuntime() = default;

bool ForkliftPduRuntime::load_pad(HakoCpp_GameControllerOperation& out_pad)
{
    return impl_->load_pad(out_pad);
}

void ForkliftPduRuntime::publish_state(
    hako::robots::controller::ForkliftController& controller,
    const HakoniwaMujocoContext::ControlState& control_state)
{
    impl_->publish_state(controller, control_state);
}
