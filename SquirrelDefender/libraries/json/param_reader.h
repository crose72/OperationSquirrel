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
#include <string>
#include <jsoncpp/json/json.h> //sudo apt-get install libjsoncpp-dev THEN target_link_libraries(your_executable_name jsoncpp)

/********************************************************************************
 * Typedefs / Enums / Structs
 ********************************************************************************/

/********************************************************************************
 * Function prototypes and Class Definitions
 ********************************************************************************/
class ParamReader
{
public:
    ParamReader(const std::string &filename);
    ~ParamReader();

    float get_float_param(const std::string &group) const;
    uint32_t get_uint32_param(const std::string &group) const;
    int get_int_param(const std::string &group) const;
    bool get_bool_param(const std::string &group) const;
    std::string get_string_param(const std::string &group) const;

private:
    Json::Value root;

    const Json::Value *resolve_path(const std::string &path) const;
};

#endif // JSON_UTILS_H