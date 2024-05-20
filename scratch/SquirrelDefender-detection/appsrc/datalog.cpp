/********************************************************************************
 * @file    datalog.cpp
 * @author  Cameron Rose
 * @date    12/27/2023
 * @brief   Record key information in a text file for debugging and issue
 *          resolution.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "datalog.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
bool headingsWritten = false;
std::string dataFileName = "data";
std::string unusedDataFileName = "";
std::vector<std::vector<std::string>> data = {};

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: toString
 * Description: Convert a value to a string (float, integer, etc.)..
 ********************************************************************************/
template <typename T>
std::string toString(const T& value) 
{
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

/********************************************************************************
 * Function: logData
 * Description: Determine when to log flight data and then log it.
 ********************************************************************************/
void logData(void)
{
    if (firstLoopAfterStartup == true)
    {
        unusedDataFileName = checkAndAppendFileName(dataFileName);
    }

    if (headingsWritten == false)
    {
        // Heading row
        data.push_back({"Time",
                        "Latitude",
                        "Longtitude",
                        "Altitude",
                        "Relative Altitude",
                        "GPS Vx",
                        "GPS Vy",
                        "GPS Vz",
                        "Heading",
                        "Roll",
                        "Pitch",
                        "Yaw",
                        "Rollspeed",
                        "Pitchspeed",
                        "YawSpeed",
                        "Accel X",
                        "Accel Y",
                        "Accel Z",
                        "Gyro X",
                        "Gyro Y",
                        "Gyro Z",
                        "Magnetic Field X",
                        "Magnetic Field Y",
                        "Magnetic Field Z"});
        writeToCSV(unusedDataFileName, data);
        headingsWritten = true;
    }

    // Clear data vector and write to next row
    data.clear();
    data.push_back({{toString(elapsedTimeSeconds),
                    toString(mav_veh_lat),
                    toString(mav_veh_lon),
                    toString(mav_veh_alt),
                    toString(mav_rel_alt),
                    toString(mav_veh_gps_vx),
                    toString(mav_veh_gps_vy),
                    toString(mav_veh_gps_vz),
                    toString(mav_veh_gps_hdg),
                    toString(mav_veh_roll),
                    toString(mav_veh_pitch),
                    toString(mav_veh_yaw),
                    toString(mav_veh_rollspeed),
                    toString(mav_veh_pitchspeed),
                    toString(mav_veh_yawspeed),
                    toString(mav_veh_imu_ax),
                    toString(mav_veh_imu_ay),
                    toString(mav_veh_imu_az),
                    toString(mav_veh_imu_xgyro),
                    toString(mav_veh_imu_ygyro),
                    toString(mav_veh_imu_zgyro),
                    toString(mav_veh_imu_xmag),
                    toString(mav_veh_imu_ymag),
                    toString(mav_veh_imu_zmag)}});
    writeToCSV(unusedDataFileName, data);
}

/********************************************************************************
 * Function: writeToCSV
 * Description: Write the data passed to this function into a CSV.
 ********************************************************************************/
void writeToCSV(const std::string& filename, const std::vector<std::vector<std::string>>& data) 
{
    std::ofstream outFile(filename, std::ios_base::app); // Open in append mode
    if (!outFile.is_open()) 
    {
        std::cerr << "Error: Unable to open file '" << filename << "'" << std::endl;
        return;
    }

    // Write data to the file, column-wise
    if (!data.empty()) 
    {
        size_t numRows = data.size();
        size_t numCols = data[0].size();

        for (size_t row = 0; row < numRows; ++row) 
        {
            for (size_t col = 0; col < numCols; ++col) 
            {
                outFile << data[row][col];
                if (col < numCols - 1) 
                {
                    outFile << ',';
                }
            }
            outFile << '\n';
        }
    }

    outFile.close();
    // std::cout << "File " << filename << " written successfully." << std::endl;
}

/********************************************************************************
 * Function: checkAndAppendFileName
 * Description: Search for existing file name and increment the file number to 1
 *              greater than the highest existing file number.
 ********************************************************************************/
std::string checkAndAppendFileName(const std::string& filename) 
{
    int counter = 1;
    std::string newFilename = filename + ".csv"; // Add .csv extension initially
    std::ifstream file(newFilename);

    while (file.is_open() == true) 
    {
        file.close();
        // If the file exists, append the counter number and try again
        newFilename = filename + "_" + std::to_string(counter++) + ".csv";
        file.open(newFilename);
    }

    return newFilename;
}
