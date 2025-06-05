#if defined(BLD_JETSON_B01) || defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

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
 * Function: ParamReader
 * Description: Class constructor
 ********************************************************************************/
ParamReader::ParamReader(const std::string &filename)
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
 * Function: ~ParamReader
 * Description: Class destructor
 ********************************************************************************/
ParamReader::~ParamReader(void) {}

/********************************************************************************
 * Function: get_float_params
 * Description: Return the values of parameters that are of type float.
 ********************************************************************************/
float ParamReader::get_float_param(const std::string &group, const std::string &key) const
{
    return root[group][key].asFloat();
}

/********************************************************************************
 * Function: get_uint32_params
 * Description: Return the values of parameters that are of type uint32_t.
 ********************************************************************************/
uint32_t ParamReader::get_uint32_param(const std::string &group, const std::string &key) const
{
    return root[group][key].asUInt();
}

/********************************************************************************
 * Function: get_bool_params
 * Description: Return the values of parameters that are of type bool.
 ********************************************************************************/
bool ParamReader::get_bool_param(const std::string &group, const std::string &key) const
{
    return root[group][key].asBool();
}

#endif // BLD_JETSON_B01 || BLD_JETSON_ORIN_NANO
