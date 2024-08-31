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
std::string dataFilePath = "../data/";
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
DataLogger::DataLogger() {};

/********************************************************************************
 * Function: ~DataLogger
 * Description: Class destructor
 ********************************************************************************/
DataLogger::~DataLogger() {};

/********************************************************************************
 * Function: generate_unique_filename
 * Description: If file exists, append an incremented number to the file name.
 ********************************************************************************/
std::string DataLogger::generate_unique_filename(const std::string &filename)
{
    int counter = 1;
    std::string new_file_name = dataFilePath + filename + ".csv"; // Add .csv extension initially
    std::ifstream file(new_file_name);

    while (file.is_open() == true)
    {
        file.close();
        // If the file exists, append the counter number and try again
        new_file_name = dataFilePath + filename + "_" + std::to_string(counter++) + ".csv";
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

#ifdef USE_JETSON

    file_name = generate_unique_filename(dataFileName);
    data.push_back({"Time",
                    "System State",
                    "x_actual",
                    "height_actual",
                    "y_actual",
                    "width_actual",
                    "x_centroid_err",
                    "target_height_err",
                    "y_centroid_err",
                    "target_left_side",
                    "target_right_side",
                    "target_left_err",
                    "target_right_err",
                    "target_height_err_rev",
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
                    "Flow Rate Y",
                    "NED X",
                    "NED Y",
                    "NED Z",
                    "NED Vx",
                    "NED Vy",
                    "NED Vz",
                    "mav_veh_q1_actual",
                    "mav_veh_q2_actual",
                    "mav_veh_q3_actual",
                    "mav_veh_q4_actual",
                    "mav_veh_roll_rate_actual",
                    "mav_veh_pitch_rate_actual",
                    "mav_veh_yaw_rate_actuald",
                    "mav_veh_repr_offset_q[0]",
                    "mav_veh_repr_offset_q[1]",
                    "mav_veh_repr_offset_q[2]",
                    "mav_veh_repr_offset_q[3]"});

#elif USE_WSL

    data.push_back({"Time",
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
                    "Flow Rate Y",
                    "NED X",
                    "NED Y",
                    "NED Z",
                    "NED Vx",
                    "NED Vy",
                    "NED Vz",
                    "mav_veh_q1_actual",
                    "mav_veh_q2_actual",
                    "mav_veh_q3_actual",
                    "mav_veh_q4_actual",
                    "mav_veh_roll_rate_actual",
                    "mav_veh_pitch_rate_actual",
                    "mav_veh_yaw_rate_actuald",
                    "mav_veh_repr_offset_q[0]",
                    "mav_veh_repr_offset_q[1]",
                    "mav_veh_repr_offset_q[2]",
                    "mav_veh_repr_offset_q[3]"});

#endif // USE_JETSON

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

#ifdef USE_JETSON

    data.push_back({{std::to_string(app_elapsed_time),
                     std::to_string(system_state),
                     std::to_string(x_actual),
                     std::to_string(height_actual),
                     std::to_string(y_actual),
                     std::to_string(width_actual),
                     std::to_string(x_centroid_err),
                     std::to_string(target_height_err),
                     std::to_string(y_centroid_err),
                     std::to_string(target_left_side),
                     std::to_string(target_right_side),
                     std::to_string(target_left_err),
                     std::to_string(target_right_err),
                     std::to_string(target_height_err_rev),
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
                     std::to_string(mav_veh_flow_rate_y),
                     std::to_string(mav_veh_local_ned_x),
                     std::to_string(mav_veh_local_ned_y),
                     std::to_string(mav_veh_local_ned_z),
                     std::to_string(mav_veh_local_ned_vx),
                     std::to_string(mav_veh_local_ned_vy),
                     std::to_string(mav_veh_local_ned_vz),
                     std::to_string(mav_veh_q1_actual),
                     std::to_string(mav_veh_q2_actual),
                     std::to_string(mav_veh_q3_actual),
                     std::to_string(mav_veh_q4_actual),
                     std::to_string(mav_veh_roll_rate_actual),
                     std::to_string(mav_veh_pitch_rate_actual),
                     std::to_string(mav_veh_yaw_rate_actual),
                     std::to_string(mav_veh_repr_offset_q[0]),
                     std::to_string(mav_veh_repr_offset_q[1]),
                     std::to_string(mav_veh_repr_offset_q[2]),
                     std::to_string(mav_veh_repr_offset_q[3])}});

#elif USE_WSL

    data.push_back({{std::to_string(app_elapsed_time),
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
                     std::to_string(mav_veh_flow_rate_y),
                     std::to_string(mav_veh_local_ned_x),
                     std::to_string(mav_veh_local_ned_y),
                     std::to_string(mav_veh_local_ned_z),
                     std::to_string(mav_veh_local_ned_vx),
                     std::to_string(mav_veh_local_ned_vy),
                     std::to_string(mav_veh_local_ned_vz),
                     std::to_string(mav_veh_q1_actual),
                     std::to_string(mav_veh_q2_actual),
                     std::to_string(mav_veh_q3_actual),
                     std::to_string(mav_veh_q4_actual),
                     std::to_string(mav_veh_roll_rate_actual),
                     std::to_string(mav_veh_pitch_rate_actual),
                     std::to_string(mav_veh_yaw_rate_actual),
                     std::to_string(mav_veh_repr_offset_q[0]),
                     std::to_string(mav_veh_repr_offset_q[1]),
                     std::to_string(mav_veh_repr_offset_q[2]),
                     std::to_string(mav_veh_repr_offset_q[3])}});

#endif // USE_JETSON

    save_to_csv(file_name, data);
}