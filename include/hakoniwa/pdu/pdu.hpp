#pragma once

#include <string>
#include <iostream>
#include "include/hako_asset.h"
#include "pdu_primitive_ctypes.h"
#include "pdu_convertor.hpp"
#include "include/hako_asset_pdu.hpp"

namespace hako::robots::pdu {
    template<typename BaseType, typename CppType, typename Convertor>
    class PDU {
        protected:
            std::string robotName;
            int channelId;
            int pduSize = -1;
            std::string pduName;
            void* top_ptr;
            char* base_ptr;

        public:
            PDU(std::string& robot_name, std::string& pdu_name, int heap_size = 0)
                : robotName(robot_name), pduName(pdu_name)
            {
                top_ptr = nullptr;
                base_ptr = nullptr;
                convertor_ = hako::pdu::PduConvertor<CppType, Convertor>();
                bool ret = hako_asset_get_pdus(robots_);
                if (!ret) {
                    std::cerr << "[ERROR] Failed to get PDU configuration." << std::endl;
                    throw std::runtime_error("Failed to get PDU configuration");
                }
                else {
                    std::cout << "[INFO] PDU configuration loaded. robotName=" << robotName << " pdu_name=" << pduName << std::endl;
                }
                if (!getPduInfo(pduSize, channelId)) {
                    std::cerr << "[ERROR] PDU not found in configuration: robotName=" << robotName << " pdu_name=" << pduName << std::endl;
                    pduSize = -1;
                    throw std::invalid_argument("PDU not found in configuration");
                }
                else {
                    std::cout << "[INFO] PDU size: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                    char* base_ptr = (char*)hako_create_empty_pdu(sizeof(BaseType), heap_size);
                    if (base_ptr == nullptr) {
                        std::cerr << "[ERROR] Failed to allocate memory for PDU: size=" << pduSize << std::endl;
                        pduSize = -1;
                        throw std::bad_alloc();
                    }
                    else {
                        top_ptr = hako_get_top_ptr_pdu(base_ptr);
                    }
                }
            }
            virtual ~PDU() 
            {
                if (base_ptr != nullptr) {
                    hako_destroy_pdu(base_ptr);
                }
            }
            bool load(CppType& pdu_cpp_data) 
            {
                if (hako_asset_pdu_read(robotName.c_str(), channelId, (char*)top_ptr, static_cast<size_t>(pduSize)) != 0) {
                    std::cerr << "Failed to read PDU data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                    return false;
                }
                if (!convertor_.pdu2cpp((char*)top_ptr, pdu_cpp_data)) {
                    std::cerr << "Failed to convert PDU to C++ data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                    return false;
                }
                return true;
            }
            bool flush(CppType& pdu_cpp_data) 
            {
                int pdu_size = convertor_.cpp2pdu(pdu_cpp_data, (char*)top_ptr, pduSize);
                if (pdu_size <= 0) {
                    std::cerr << "Failed to convert C++ data to PDU: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pduSize << std::endl;
                    return false;
                }
                if (pdu_size != pduSize) {
                    std::cerr << "PDU size mismatch: robotName=" << robotName << " channelId=" << channelId << " expectedSize=" << pduSize << " actualSize=" << pdu_size << std::endl;
                    return false;
                }
                if (hako_asset_pdu_write(robotName.c_str(), channelId, (char*)top_ptr, static_cast<size_t>(pdu_size)) != 0) {
                    std::cerr << "Failed to write PDU data: robotName=" << robotName << " channelId=" << channelId << " pduSize=" << pdu_size << std::endl;
                    return false;
                }
                return true;
            }            
        private:
            hako::pdu::PduConvertor<CppType, Convertor> convertor_;
            std::vector<hako::asset::Robot> robots_;
            bool getPduInfo(int& pdu_size, int& channel_id) const 
            {
                for (const auto& robot : robots_) {
                    //std::cout << "[DEBUG] Checking robot: " << robot.name << std::endl;
                    if (robot.name == robotName) {
                        //std::cout << "[DEBUG] Found matching robot: " << robot.name << std::endl;
                        for (const auto& writer : robot.pdu_writers) {
                            //std::cout << "[DEBUG] Checking writer: " << writer.name << " (org_name: " << writer.org_name << ", channel_id: " << writer.channel_id << ")" << std::endl;
                            if (writer.org_name == pduName) {
                                pdu_size = writer.pdu_size;
                                channel_id = writer.channel_id;
                                return true;
                            }
                        }
                        for (const auto& reader : robot.pdu_readers) {
                            //std::cout << "[DEBUG] Checking reader: " << reader.name << " (org_name: " << reader.org_name << ", channel_id: " << reader.channel_id << ")" << std::endl;
                            if (reader.org_name == pduName) {
                                pdu_size = reader.pdu_size;
                                channel_id = reader.channel_id;
                                return true;
                            }
                        }
                    }
                }
                return false;
            }
    };
}
    
#define DECLARE_PDU_INSTANCE(pdu_package, pdu_type, instance, robot_name, pdu_name) \
    hako::robots::pdu::PDU< \
        Hako_##pdu_type, \
        HakoCpp_##pdu_type, \
        pdu_package::pdu_type \
    > instance(robot_name, pdu_name)
