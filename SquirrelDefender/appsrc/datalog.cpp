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
                    "Onboard Control Sensors Present",
                    "Onboard Control Sensors Enabled",
                    "Onboard Control Sensors Health",
                    "System Load",
                    "Voltage Battery",
                    "Current Battery",
                    "Drop Rate Comm",
                    "Errors Comm",
                    "Errors Count 1",
                    "Errors Count 2",
                    "Errors Count 3",
                    "Errors Count 4",
                    "Battery Remaining",
                    "Onboard Control Sensors Present Extended",
                    "Onboard Control Sensors Enabled Extended",
                    "Onboard Control Sensors Health Extended",
                    "Custom Mode",
                    "Vehicle Type",
                    "Autopilot Type",
                    "Base Mode",
                    "Vehicle State",
                    "MAVLink Version",
                    "Latitude",
                    "Longitude",
                    "Altitude",
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
                    "IMU Mag X",
                    "IMU Mag Y",
                    "IMU Mag Z",
                    "Target Quaternion 1",
                    "Target Quaternion 2",
                    "Target Quaternion 3",
                    "Target Quaternion 4",
                    "Target Roll Rate",
                    "Target Pitch Rate",
                    "Target Yaw Rate",
                    "Target Thrust",
                    "Actual Quaternion 1",
                    "Actual Quaternion 2",
                    "Actual Quaternion 3",
                    "Actual Quaternion 4",
                    "Actual Roll Rate",
                    "Actual Pitch Rate",
                    "Actual Yaw Rate",
                    "Actual Thrust",
                    "Rangefinder Min Distance",
                    "Rangefinder Max Distance",
                    "Rangefinder Current Distance",
                    "Rangefinder Type",
                    "Rangefinder ID",
                    "Rangefinder Orientation",
                    "Rangefinder Covariance",
                    "Rangefinder Horizontal FOV",
                    "Rangefinder Vertical FOV",
                    "Rangefinder Quaternion 1",
                    "Rangefinder Quaternion 2",
                    "Rangefinder Quaternion 3",
                    "Rangefinder Quaternion 4",
                    "Rangefinder Signal Quality",
                    "Optical Flow Compensated X",
                    "Optical Flow Compensated Y",
                    "Ground Distance",
                    "Optical Flow X",
                    "Optical Flow Y",
                    "Sensor ID",
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
                     std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_present),
                     std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_enabled),
                     std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_health),
                     std::to_string(mav_veh_sys_stat_load),
                     std::to_string(mav_veh_sys_stat_voltage_battery),
                     std::to_string(mav_veh_sys_stat_current_battery),
                     std::to_string(mav_veh_sys_stat_drop_rate_comm),
                     std::to_string(mav_veh_sys_stat_errors_comm),
                     std::to_string(mav_veh_sys_stat_errors_count1),
                     std::to_string(mav_veh_sys_stat_errors_count2),
                     std::to_string(mav_veh_sys_stat_errors_count3),
                     std::to_string(mav_veh_sys_stat_errors_count4),
                     std::to_string(mav_veh_sys_stat_battery_remaining),
                     std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_prsnt_extnd),
                     std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_enbld_extnd),
                     std::to_string(mav_veh_sys_stat_onbrd_cntrl_snsrs_health_extnd),
                     std::to_string(mav_veh_custom_mode),
                     std::to_string(mav_veh_type),
                     std::to_string(mav_veh_autopilot_type),
                     std::to_string(mav_veh_base_mode),
                     std::to_string(mav_veh_state),
                     std::to_string(mav_veh_mavlink_version),
                     std::to_string(mav_veh_lat),
                     std::to_string(mav_veh_lon),
                     std::to_string(mav_veh_alt),
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
                     std::to_string(mav_veh_imu_xmag),
                     std::to_string(mav_veh_imu_ymag),
                     std::to_string(mav_veh_imu_zmag),
                     std::to_string(mav_veh_q1_target),
                     std::to_string(mav_veh_q2_target),
                     std::to_string(mav_veh_q3_target),
                     std::to_string(mav_veh_q4_target),
                     std::to_string(mav_veh_roll_rate_target),
                     std::to_string(mav_veh_pitch_rate_target),
                     std::to_string(mav_veh_yaw_rate_target),
                     std::to_string(mav_veh_thrust_target),
                     std::to_string(mav_veh_q1_actual),
                     std::to_string(mav_veh_q2_actual),
                     std::to_string(mav_veh_q3_actual),
                     std::to_string(mav_veh_q4_actual),
                     std::to_string(mav_veh_roll_rate_actual),
                     std::to_string(mav_veh_pitch_rate_actual),
                     std::to_string(mav_veh_yaw_rate_actual),
                     std::to_string(mav_veh_thrust_actual),
                     std::to_string(mav_veh_rngfdr_min_distance),
                     std::to_string(mav_veh_rngfdr_max_distance),
                     std::to_string(mav_veh_rngfdr_current_distance),
                     std::to_string(mav_veh_rngfdr_type),
                     std::to_string(mav_veh_rngfdr_id),
                     std::to_string(mav_veh_rngfdr_orientation),
                     std::to_string(mav_veh_rngfdr_covariance),
                     std::to_string(mav_veh_rngfdr_horizontal_fov),
                     std::to_string(mav_veh_rngfdr_vertical_fov),
                     std::to_string(mav_veh_rngfdr_quaternion[0]),
                     std::to_string(mav_veh_rngfdr_quaternion[1]),
                     std::to_string(mav_veh_rngfdr_quaternion[2]),
                     std::to_string(mav_veh_rngfdr_quaternion[3]),
                     std::to_string(mav_veh_rngfdr_signal_quality),
                     std::to_string(mav_veh_flow_comp_m_x),
                     std::to_string(mav_veh_flow_comp_m_y),
                     std::to_string(mav_veh_ground_distance),
                     std::to_string(mav_veh_flow_x),
                     std::to_string(mav_veh_flow_y),
                     std::to_string(mav_veh_sensor_id),
                     std::to_string(mav_veh_quality),
                     std::to_string(mav_veh_flow_rate_x),
                     std::to_string(mav_veh_flow_rate_y)}});
    save_to_csv(file_name, data);
}