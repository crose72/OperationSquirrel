#ifdef USE_JETSON

/********************************************************************************
* @file    parameters.cpp
* @author  Cameron Rose
* @date    12/27/2023
* @brief   Provide utilities for accessing various parameters from a json or
* 			other type of file.
********************************************************************************/

/********************************************************************************
* Includes
********************************************************************************/
#include "parameters.h"

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
* Function: Parameters
* Description: Class constructor
********************************************************************************/
Parameters::Parameters(const std::string& filename) 
{
	std::ifstream configFile(filename);

	if (!configFile.is_open()) 
	{
		std::cerr << "Unable to open file: " << filename << std::endl;
		return;
	}
	configFile >> root;
}

/********************************************************************************
* Function: ~Parameters
* Description: Class destructor
********************************************************************************/
Parameters::~Parameters(void){};

/********************************************************************************
* Function: get_float_params
* Description: Return the values of parameters that are of type float.
********************************************************************************/
float Parameters::get_float_param(const std::string& group, const std::string& key) const 
{
	return root[group][key].asFloat();
}

/********************************************************************************
* Function: get_uint32_params
* Description: Return the values of parameters that are of type uint32_t.
********************************************************************************/
uint32_t Parameters::get_uint32_param(const std::string& group, const std::string& key) const 
{
	return root[group][key].asUInt();
}

/********************************************************************************
* Function: get_bool_params
* Description: Return the values of parameters that are of type bool.
********************************************************************************/
bool Parameters::get_bool_param(const std::string& group, const std::string& key) const 
{
	return root[group][key].asBool();
}

#endif // USE_JETSON