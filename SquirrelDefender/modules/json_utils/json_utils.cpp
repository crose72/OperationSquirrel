#if defined(BLD_JETSON_B01) || defined(BLD_JETSON_ORIN_NANO)

/********************************************************************************
 * @file    json_utils.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Provide utilities for accessing parameters from a json file.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "json_utils.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: JSONUtils
 * Description: Class constructor
 ********************************************************************************/
JSONUtils::JSONUtils(const std::string &filename)
{
    std::ifstream configFile(filename);

    if (!configFile.is_open())
    {
        std::cerr << "Error: Unable to open parameter file: " << filename << std::endl;
        return;
    }
    configFile >> root;
}

/********************************************************************************
 * Function: ~JSONUtils
 * Description: Class destructor
 ********************************************************************************/
JSONUtils::~JSONUtils(void) {}

/********************************************************************************
 * Function: get_float_params
 * Description: Return the values of parameters that are of type float.
 ********************************************************************************/
float JSONUtils::get_float_param(const std::string &group, const std::string &key) const
{
    return root[group][key].asFloat();
}

/********************************************************************************
 * Function: get_uint32_params
 * Description: Return the values of parameters that are of type uint32_t.
 ********************************************************************************/
uint32_t JSONUtils::get_uint32_param(const std::string &group, const std::string &key) const
{
    return root[group][key].asUInt();
}

/********************************************************************************
 * Function: get_bool_params
 * Description: Return the values of parameters that are of type bool.
 ********************************************************************************/
bool JSONUtils::get_bool_param(const std::string &group, const std::string &key) const
{
    return root[group][key].asBool();
}

#endif // BLD_JETSON_B01 || BLD_JETSON_ORIN_NANO
