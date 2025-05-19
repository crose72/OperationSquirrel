/********************************************************************************
 * @file    test_csv_utils.h
 * @author  Cameron Rose
 * @date    2/18/2025
 ********************************************************************************/
#ifndef TEST_CSV_UTILS_H
#define TEST_CSV_UTILS_H

/********************************************************************************
 * Includes
 ********************************************************************************/
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>

/********************************************************************************
 * Imported objects
 ********************************************************************************/

/********************************************************************************
 * Exported objects
 ********************************************************************************/

/********************************************************************************
 * Function prototypes
 ********************************************************************************/
int find_column_index(const std::string& header, const std::string& column);

/********************************************************************************
 * Function: convert_value
 * Description: Convert string to a numeric data type.
 ********************************************************************************/
// Template function to convert string to the desired type
template <typename T>
T convert_value(const std::string& value) 
{
    if constexpr (std::is_same_v<T, std::string>) 
    {
        return value;
    } else 
    {
        try
        {
            if constexpr (std::is_same_v<T, int>) 
            {
                return std::stoi(value);
            } else if constexpr (std::is_same_v<T, float>) 
            {
                return std::stof(value);
            } else if constexpr (std::is_same_v<T, double>) 
            {
                return std::stod(value);
            }
        } 
        catch (const std::exception& e) 
        {
            std::cerr << "Warning: Could not convert value '" << value << "' to the requested type. Error: " << e.what() << std::endl;
        }
    }
    return T{};  // Return default value of type T in case of conversion failure
}

/********************************************************************************
 * Function: get_column_data
 * Description: Return a vector of all of the data in a specific column of a csv
 *              file, given the header of the column (e.g create an array of all
 *              timestamps recorded in the "time" column).
 ********************************************************************************/
// Templated function to extract column data of any type
template <typename T>
std::vector<T> get_column_data(const std::string& file, const std::string& column) 
{
    std::ifstream input_file(file);
    std::vector<T> column_data;

    if (!input_file.is_open()) 
    {
        std::cerr << "Error opening file: " << file << std::endl;
        return column_data;
    }

    std::string header;
    std::getline(input_file, header);  // Read header

    int column_index = find_column_index(header, column);
    if (column_index == -1) 
    {
        std::cerr << "Error: Column '" << column << "' not found!" << std::endl;
        return column_data;
    }

    std::string line;
    while (std::getline(input_file, line)) 
    {
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> row_values;

        while (std::getline(ss, value, ',')) 
        {
            row_values.push_back(value);
        }

        if (row_values.size() <= column_index)
        {
            continue;  // Skip invalid rows
        }

        column_data.push_back(convert_value<T>(row_values[column_index]));  // Convert and store the value
    }

    input_file.close();
    return column_data;
}

std::string generate_unique_filename(const std::string &data_file_path, const std::string &filename);

#endif // TEST_CSV_UTILS_H
