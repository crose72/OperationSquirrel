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
        return 0.0f;
    return node->asFloat();
}

/********************************************************************************
 * Function: get_uint32_params
 * Description: Return the values of parameters that are of type uint32_t.
 ********************************************************************************/
uint32_t ParamReader::get_uint32_param(const std::string &path) const
{
    const Json::Value *node = resolve_path(path);
    if (!node)
        return 0u;
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
        return 0;
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
