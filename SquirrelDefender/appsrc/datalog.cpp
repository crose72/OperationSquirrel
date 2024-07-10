/********************************************************************************
 * @file    datalog.cpp
 * @author  Cameron Rose
 * @date    6/7/2023
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
bool headings_written = false;
std::string dataFileName = "data";
std::string file_name = "";
std::vector<std::vector<std::string>> data = {};

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/

/********************************************************************************
 * Function: DataLogger
 * Description: Class constructor
 ********************************************************************************/
DataLogger::DataLogger(){};

/********************************************************************************
 * Function: ~DataLogger
 * Description: Class destructor
 ********************************************************************************/
DataLogger::~DataLogger(){};

/********************************************************************************
 * Function: generate_unique_filename
 * Description: If file exists, append an incremented number to the file name.
 ********************************************************************************/
std::string DataLogger::generate_unique_filename(const std::string &filename)
{
    int counter = 1;
    std::string new_file_name = filename + ".csv"; // Add .csv extension initially
    std::ifstream file(new_file_name);

    while (file.is_open() == true)
    {
        file.close();
        // If the file exists, append the counter number and try again
        new_file_name = filename + "_" + std::to_string(counter++) + ".csv";
        file.open(new_file_name);
    }

    return new_file_name;
}

/********************************************************************************
 * Function: save_to_csv
 * Description: Write the data passed to this function into a CSV.
 ********************************************************************************/
void DataLogger::save_to_csv(const std::string &filename, const std::vector<std::vector<std::string>> &data)
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
}

/********************************************************************************
 * Function: data_log_init
 * Description: Initialize data log variables and files.
 ********************************************************************************/
bool DataLogger::data_log_init(void)
{
    file_name = generate_unique_filename(dataFileName);
    data.push_back({"Time",
                    "Target identified",
                    "Target too close",
                    "Vx adjust",
                    "Vy adjust",
                    "Vz adjust",
                    "Voltage Battery",
                    "Current Battery",
                    "Battery Remaining",
                    "Relative Altitude",
                    "GPS Vx",
                    "GPS Vy",
                    "GPS Vz",
                    "GPS Heading",
                    "Roll",
                    "Pitch",
                    "Yaw",
                    "Rollspeed",
                    "Pitchspeed",
                    "Yawspeed",
                    "IMU Accel X",
                    "IMU Accel Y",
                    "IMU Accel Z",
                    "IMU Gyro X",
                    "IMU Gyro Y",
                    "IMU Gyro Z",
                    "Rangefinder Current Distance",
                    "Rangefinder Signal Quality",
                    "Optical Flow Compensated X",
                    "Optical Flow Compensated Y",
                    "Optical Flow X",
                    "Optical Flow Y",
                    "Optical Flow Quality",
                    "Flow Rate X",
                    "Flow Rate Y"});
    save_to_csv(file_name, data);

    return true;
}

/********************************************************************************
 * Function: data_log_loop
 * Description: Log data.
 ********************************************************************************/
void DataLogger::data_log_loop(void)
{
    // Clear data vector and write to next row
    data.clear();
    data.push_back({{std::to_string(app_elapsed_time),
                     std::to_string(target_identified),
                     std::to_string(target_too_close),
                     std::to_string(vx_adjust),
                     std::to_string(vy_adjust),
                     std::to_string(vz_adjust),
                     std::to_string(mav_veh_sys_stat_voltage_battery),
                     std::to_string(mav_veh_sys_stat_current_battery),
                     std::to_string(mav_veh_sys_stat_battery_remaining),
                     std::to_string(mav_veh_rel_alt),
                     std::to_string(mav_veh_gps_vx),
                     std::to_string(mav_veh_gps_vy),
                     std::to_string(mav_veh_gps_vz),
                     std::to_string(mav_veh_gps_hdg),
                     std::to_string(mav_veh_roll),
                     std::to_string(mav_veh_pitch),
                     std::to_string(mav_veh_yaw),
                     std::to_string(mav_veh_rollspeed),
                     std::to_string(mav_veh_pitchspeed),
                     std::to_string(mav_veh_yawspeed),
                     std::to_string(mav_veh_imu_ax),
                     std::to_string(mav_veh_imu_ay),
                     std::to_string(mav_veh_imu_az),
                     std::to_string(mav_veh_imu_xgyro),
                     std::to_string(mav_veh_imu_ygyro),
                     std::to_string(mav_veh_imu_zgyro),
                     std::to_string(mav_veh_rngfdr_current_distance),
                     std::to_string(mav_veh_rngfdr_signal_quality),
                     std::to_string(mav_veh_flow_comp_m_x),
                     std::to_string(mav_veh_flow_comp_m_y),
                     std::to_string(mav_veh_flow_x),
                     std::to_string(mav_veh_flow_y),
                     std::to_string(mav_veh_quality),
                     std::to_string(mav_veh_flow_rate_x),
                     std::to_string(mav_veh_flow_rate_y)}});
    save_to_csv(file_name, data);
}