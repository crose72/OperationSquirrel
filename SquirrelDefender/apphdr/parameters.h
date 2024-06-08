#pragma once

#ifdef USE_JETSON

/********************************************************************************
 * @file    parameters.h
 * @author  Cameron Rose
 * @date    6/7/2023
 ********************************************************************************/
#ifndef PARAMETERS_H
#define PARAMETERS_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "common_inc.h"
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
class Parameters {
    public:
        Parameters(const std::string& filename);
        ~Parameters();

        float get_float_param(const std::string& group, const std::string& key) const;
        uint32_t get_uint32_param(const std::string& group, const std::string& key) const;
        bool get_bool_param(const std::string& group, const std::string& key) const;

    private:
        Json::Value root;
};


#endif // PARAMETERS_H

#endif // USE_JETSON