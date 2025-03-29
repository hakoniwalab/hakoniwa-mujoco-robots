#pragma once

#include "hakoniwa/pdu/pdu.hpp"
#include "geometry_msgs/pdu_cpptype_conv_Twist.hpp"

namespace hako::pdu::msgs::geometry_msgs {
    class Twist: public hako::robots::pdu::PDU {
        private:
            HakoCpp_Twist cppData;
        public:
            Twist(std::string& robot_name, int channel_id, int heap_size = 0)
                : PDU(robot_name, channel_id)
            {
                base_ptr = (char*)hako_create_empty_pdu_Twist(heap_size);
                if (base_ptr == nullptr) {
                    std::cerr << "Failed to create PDU data: robotName=" << robot_name << " channelId=" << channel_id << std::endl;
                }
                top_ptr = hako_get_top_ptr_pdu(base_ptr);
                if (top_ptr == nullptr) {
                    std::cerr << "Failed to get top pointer of PDU data: robotName=" << robot_name << " channelId=" << channel_id << std::endl;
                }
                //std::cout << "pduData: " << std::hex << (uintptr_t)pduData << std::dec << std::endl;
            }
            virtual bool load() override {
                if (PDU::load())
                {
                    if (hako_convert_pdu2cpp_Twist(*(Hako_Twist*)base_ptr, cppData) != 0) {
                        //std::cerr << "Failed to convert PDU data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                        return false;
                    }
                    //std::cout << "successfully load PDU data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                }
                else {
                    //std::cerr << "Failed to load PDU data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                    return false;
                }
                return true;
            }
            
            virtual bool flush()
            {
                char* pdu_msg = nullptr;;
                int pdu_size = hako_convert_cpp2pdu_Twist(cppData, (Hako_Twist**)&pdu_msg);
                if (pdu_size < 0) {
                    std::cerr << "Failed to convert PDU data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                    return false;
                }
                auto ret = PDU::flush(pdu_msg, pdu_size);
                hako_destroy_pdu(pdu_msg); 
                return ret;
            }

            virtual ~Twist() {}
            HakoCpp_Twist& getData() {
                return cppData;
            }
            const HakoCpp_Twist& getData() const {
                return cppData;
            }
    };
}
