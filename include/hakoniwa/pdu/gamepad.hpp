#pragma once

#include "hakoniwa/pdu/pdu.hpp"
#include "pdu/types/hako_msgs/pdu_cpptype_conv_GameControllerOperation.hpp"
#include "include/hako_asset.h"

namespace hako::robots::pdu {
    class GamePad: public PDU {
        private:
            HakoCpp_GameControllerOperation cppData;
        public:
            GamePad(std::string& robot_name, int channel_id)
                : PDU(robot_name, channel_id)
            {
                int heap_size = 0;
                pduData = (char*)hako_create_empty_pdu_GameControllerOperation(heap_size);
                if (pduData == nullptr) {
                    std::cerr << "Failed to create PDU data: robotName=" << robot_name << " channelId=" << channel_id << std::endl;
                }
                else if (load())
                {
                    if (hako_convert_pdu2cpp_GameControllerOperation(*(Hako_GameControllerOperation*)pduData, cppData) != 0) {
                        std::cerr << "Failed to convert PDU data: robotName=" << robot_name << " channelId=" << channel_id << " pduSize=" << pduSize << std::endl;
                    }
                }
                else {
                    std::cerr << "Failed to load PDU data: robotName=" << robot_name << " channelId=" << channel_id << " pduSize=" << pduSize << std::endl;
                }
            }

            virtual ~GamePad() {}
            HakoCpp_GameControllerOperation& getData() {
                return cppData;
            }
            const HakoCpp_GameControllerOperation& getData() const {
                return cppData;
            }
            void setAxis(const std::array<Hako_float64, 6>& axis) {
                cppData.axis = axis;
            }
            const std::array<Hako_float64, 6>& getAxis() const {
                return cppData.axis;
            }
            void setButton(const std::array<Hako_bool, 15>& button) {
                cppData.button = button;
            }
            const std::array<Hako_bool, 15>& getButton() const {
                return cppData.button;
            }
    };
}
