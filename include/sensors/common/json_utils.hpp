#pragma once

#include <fstream>
#include <string>

#include <nlohmann/json.hpp>

namespace hako::robots::sensor::common
{
    using json = nlohmann::json;

    inline bool load_json_file(const std::string& path, json& out)
    {
        std::ifstream ifs(path);
        if (!ifs.is_open()) {
            return false;
        }
        ifs >> out;
        return true;
    }

    inline double get_json_number(const json& j, const char* key, double default_value)
    {
        if (!j.contains(key) || !j.at(key).is_number()) {
            return default_value;
        }
        return j.at(key).get<double>();
    }

    inline int get_json_int(const json& j, const char* key, int default_value)
    {
        if (!j.contains(key) || !j.at(key).is_number_integer()) {
            return default_value;
        }
        return j.at(key).get<int>();
    }

    inline std::string get_json_string(const json& j, const char* key, const std::string& default_value)
    {
        if (!j.contains(key) || !j.at(key).is_string()) {
            return default_value;
        }
        return j.at(key).get<std::string>();
    }

    inline bool get_json_bool(const json& j, const char* key, bool default_value)
    {
        if (!j.contains(key) || !j.at(key).is_boolean()) {
            return default_value;
        }
        return j.at(key).get<bool>();
    }
}
