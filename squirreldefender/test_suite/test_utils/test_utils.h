/********************************************************************************
 * @file    test_utils.h
 * @author  Cameron Rose
 * @date    2/18/2025
 ********************************************************************************/
#ifndef TEST_UTILS_H
#define TEST_UTILS_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
std::string generate_unique_filename(const std::string &data_file_path, const std::string &filename);

#endif // TEST_UTILS_H
