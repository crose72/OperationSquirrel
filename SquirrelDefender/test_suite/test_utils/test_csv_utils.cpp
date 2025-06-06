/********************************************************************************
 * @file    test_csv_utils.cpp
 * @author  Cameron Rose
 * @date    2/18/2025
 * @brief   Utility functions for working with csv files.  For example, creating
 *          an array from a column of data in a csv.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "test_csv_utils.h"

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
 * Function: 
 * Description: 
 ********************************************************************************/

/********************************************************************************
 * Function: find_column_index
 * Description: Return the index of a column header in a csv file given the text
 *              in the column header (e.g. if "time" is header of column 5, 
 *              return 5)
 ********************************************************************************/
int find_column_index(const std::string& header, const std::string& column) 
{
    std::stringstream ss(header);
    std::string col;
    int column_index = 0;

    while (std::getline(ss, col, ',')) 
    {
        if (col == column) return column_index;
        column_index++;
    }

    return -1; // Column not found
}

/********************************************************************************
 * Function: generate_unique_filename
 * Description: If file exists, append an incremented number to the file name.
 ********************************************************************************/
std::string generate_unique_filename(const std::string &data_file_path, const std::string &filename)
{
    int counter = 1;
    std::string new_file_name = data_file_path + filename + ".csv"; // Add .csv extension initially
    std::ifstream file(new_file_name);

    while (file.is_open() == true)
    {
        file.close();
        // If the file exists, append the counter number and try again
        new_file_name = data_file_path + filename + "_" + std::to_string(counter++) + ".csv";
        file.open(new_file_name);
    }

    return new_file_name;
}