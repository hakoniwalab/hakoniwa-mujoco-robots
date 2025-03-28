#pragma once

#include "hakoniwa/pdu/pdu.hpp"
#include "hako_msgs/pdu_cpptype_conv_GameControllerOperation.hpp"

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
                else 
                {
                    this->load();
                }
            }
            virtual bool load() override {
                if (PDU::load())
                {
                    if (hako_convert_pdu2cpp_GameControllerOperation(*(Hako_GameControllerOperation*)pduData, cppData) != 0) {
                        std::cerr << "Failed to convert PDU data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                        return false;
                    }
                }
                else {
                    std::cerr << "Failed to load PDU data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                    return false;
                }
                return true;
            }
            
            virtual bool flush() {
                char* pdu_msg = nullptr;;
                int pdu_size = hako_convert_cpp2pdu_GameControllerOperation(cppData, (Hako_GameControllerOperation**)&pdu_msg);
                if (pdu_size < 0) {
                    std::cerr << "Failed to convert PDU data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                    return false;
                }
                auto ret = PDU::flush(pdu_msg, pdu_size);
                hako_destroy_pdu(pdu_msg); 
                return ret;
            }

            virtual ~GamePad() {}
            HakoCpp_GameControllerOperation& getData() {
                return cppData;
            }
            const HakoCpp_GameControllerOperation& getData() const {
                return cppData;
            }
    };
}
