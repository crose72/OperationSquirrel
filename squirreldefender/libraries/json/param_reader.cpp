/********************************************************************************
 * @file    param_reader.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Provide utilities for accessing parameters from a json file.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "param_reader.h"
#include <stdint.h>
#include <fstream>
#include <spdlog/spdlog.h>

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: ParamReader
 * Description: Class constructor
 ********************************************************************************/
ParamReader::ParamReader(const std::string &filename)
{
    std::ifstream configFile(filename);

    if (!configFile.is_open())
    {
        spdlog::error("Error: Unable to open parameter file: " + filename);
        return;
    }
    configFile >> root;
}

/********************************************************************************
 * Function: ~ParamReader
 * Description: Class destructor
 ********************************************************************************/
ParamReader::~ParamReader(void) {}

/********************************************************************************
 * Function: get_float_params
 * Description: Return the values of parameters that are of type float.
 ********************************************************************************/
float ParamReader::get_float_param(const std::string &path) const
{
    const Json::Value *node = resolve_path(path);
    if (!node)
        return (float)0.0;
    return node->asFloat();
}

/********************************************************************************
 * Function: get_uint8_param
 * Description: Return the values of parameters that are of type uint8_t.
 ********************************************************************************/
uint8_t ParamReader::get_uint8_param(const std::string &path) const
{
    const Json::Value *node = resolve_path(path);
    if (!node)
        return (uint8_t)0;

    uint32_t value = node->asUInt();
    if (value > UINT8_MAX)
    {
        spdlog::warn("ParamReader: Value out of uint8_t range for '{}': {}",
                     path, value);
        return UINT8_MAX;
    }

    return static_cast<uint8_t>(value);
}

/********************************************************************************
 * Function: get_uint16_param
 * Description: Return the values of parameters that are of type uint16_t.
 ********************************************************************************/
uint16_t ParamReader::get_uint16_param(const std::string &path) const
{
    const Json::Value *node = resolve_path(path);
    if (!node)
        return (uint16_t)0;

    // Clamp to uint16_t range to avoid accidental overflow
    uint32_t value = node->asUInt();
    if (value > UINT16_MAX)
    {
        spdlog::warn("ParamReader: Value out of uint16_t range for '{}': {}",
                     path, value);
        return UINT16_MAX;
    }

    return static_cast<uint16_t>(value);
}

/********************************************************************************
 * Function: get_uint32_params
 * Description: Return the values of parameters that are of type uint32_t.
 ********************************************************************************/
uint32_t ParamReader::get_uint32_param(const std::string &path) const
{
    const Json::Value *node = resolve_path(path);
    if (!node)
        return (uint32_t)0;
    return node->asUInt();
}

/********************************************************************************
 * Function: get_int_param
 * Description: Return the values of parameters that are of type int.
 ********************************************************************************/
int ParamReader::get_int_param(const std::string &path) const
{
    const Json::Value *node = resolve_path(path);
    if (!node)
        return (int)0;
    return node->asInt();
}

/********************************************************************************
 * Function: get_bool_params
 * Description: Return the values of parameters that are of type bool.
 ********************************************************************************/
bool ParamReader::get_bool_param(const std::string &path) const
{
    const Json::Value *node = resolve_path(path);
    if (!node)
        return false;
    return node->asBool();
}

/********************************************************************************
 * Function: get_string_param
 * Description: Return the value of parameters that are of type string.
 ********************************************************************************/
std::string ParamReader::get_string_param(const std::string &path) const
{
    const Json::Value *node = resolve_path(path);
    if (!node)
        return "";
    return node->asString();
}

const Json::Value *ParamReader::resolve_path(const std::string &path) const
{
    const Json::Value *node = &root;
    std::stringstream ss(path);
    std::string segment;

    while (std::getline(ss, segment, '.'))
    {
        if (!node->isMember(segment))
        {
            spdlog::error("ParamReader: Missing JSON key in path: {}", segment);
            return nullptr;
        }
        node = &(*node)[segment];
    }

    return node;
}
