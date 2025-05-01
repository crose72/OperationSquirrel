#pragma once

#if defined(BLD_JETSON_B01) || defined(BLD_JETSON_ORIN_NANO)

/********************************************************************************
 * @file    param_reader.h
 * @author  Cameron Rose
 * @date    1/22/2025
 ********************************************************************************/
#ifndef JSON_UTILS_H
#define JSON_UTILS_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <stdint.h>
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <sstream>
#include <jsoncpp/json/json.h> //sudo apt-get install libjsoncpp-dev THEN target_link_libraries(your_executable_name jsoncpp)

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class ParamReader
{
public:
    ParamReader(const std::string &filename);
    ~ParamReader();

    float get_float_param(const std::string &group, const std::string &key) const;
    uint32_t get_uint32_param(const std::string &group, const std::string &key) const;
    int get_int_param(const std::string &group, const std::string &key) const;
    bool get_bool_param(const std::string &group, const std::string &key) const;
    std::string get_string_param(const std::string &group, const std::string &key) const;

private:
    Json::Value root;
};

#endif // JSON_UTILS_H

#endif // BLD_JETSON_B01 || BLD_JETSON_ORIN_NANO
