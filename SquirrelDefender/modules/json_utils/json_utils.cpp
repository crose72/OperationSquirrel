#ifdef BLD_JETSON_B01

/********************************************************************************
 * @file    parameters.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
 * @brief   Provide utilities for accessing various parameters from a json or
 *           other type of file.
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
 * Function: json_utils
 * Description: Class constructor
 ********************************************************************************/
json_utils::json_utils(const std::string &filename)
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
 * Function: ~json_utils
 * Description: Class destructor
 ********************************************************************************/
json_utils::~json_utils(void) {}

/********************************************************************************
 * Function: get_float_params
 * Description: Return the values of parameters that are of type float.
 ********************************************************************************/
float json_utils::get_float_param(const std::string &group, const std::string &key) const
{
    return root[group][key].asFloat();
}

/********************************************************************************
 * Function: get_uint32_params
 * Description: Return the values of parameters that are of type uint32_t.
 ********************************************************************************/
uint32_t json_utils::get_uint32_param(const std::string &group, const std::string &key) const
{
    return root[group][key].asUInt();
}

/********************************************************************************
 * Function: get_bool_params
 * Description: Return the values of parameters that are of type bool.
 ********************************************************************************/
bool json_utils::get_bool_param(const std::string &group, const std::string &key) const
{
    return root[group][key].asBool();
}

#endif // BLD_JETSON_B01
