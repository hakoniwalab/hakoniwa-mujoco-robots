#pragma once

#include <string>
#include <iostream>
#include "include/hako_asset.h"
#include "pdu_primitive_ctypes.h"

namespace hako::robots::pdu {
    class PDU {
        protected:
            std::string robotName;
            int channelId;
            int pduSize = -1;
            char* pduData;
            virtual bool load() 
            {
                if (pduData == nullptr) {
                    return false;
                }
                if (pduSize < 0) {
                    setPDUSize();
                }
                if (hako_asset_pdu_read(robotName.c_str(), channelId, (char*)pduData, static_cast<size_t>(pduSize)) != 0) {
                    std::cerr << "Failed to read PDU data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                    return false;
                }
                return true;
            }
            bool flush(char* pdu_msg, int pdu_size)
            {
                if (pduData == nullptr) {
                    return false;
                }
                if (hako_asset_pdu_write(robotName.c_str(), channelId, (char*)pdu_msg, static_cast<size_t>(pdu_size)) != 0) {
                    std::cerr << "Failed to write PDU data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pdu_size << std::endl;
                    return false;
                }
                return true;
            }
        public:
            PDU(std::string& robot_name, int channel_id)
                : robotName(robot_name), channelId(channel_id)
            {
                pduData = nullptr;
            }
            void setPDUSize() 
            {
                if (pduData != nullptr) {
                    pduSize = hako_get_pdu_meta_data(pduData)->total_size;
                }
            }

            virtual ~PDU() 
            {
                if (pduData != nullptr) {
                    hako_destroy_pdu(pduData);
                }
            }
    };
}