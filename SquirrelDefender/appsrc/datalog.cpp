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
bool headings_written;
std::string data_file_path;
std::string data_file_name;
std::string file_name;
std::vector<std::vector<std::string>> data;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
static void write_headers(void);
static void save_to_csv(const std::string &filename, const std::vector<std::vector<std::string>> &data);
static std::string generate_unique_filename(const std::string &filename);

/********************************************************************************
 * Function: generate_unique_filename
 * Description: If file exists, append an incremented number to the file name.
 ********************************************************************************/
std::string generate_unique_filename(const std::string &filename)
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

/********************************************************************************
 * Function: save_to_csv
 * Description: Write the data passed to this function into a CSV.
 ********************************************************************************/
void save_to_csv(const std::string &filename, const std::vector<std::vector<std::string>> &data)
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
 * Function: write_headers
 * Description: Write the column headers of the log file.
 ********************************************************************************/
void write_headers(void)
{
    data.push_back({"app_elapsed_time",
                    "system_state",
                    "g_target_too_close",
                    "g_target_valid",
                    "target_detection_ID",
                    "target_track_ID",
                    "target_cntr_offset_x",
                    "target_cntr_offset_y",
                    "target_height",
                    "target_width",
                    "target_aspect",
                    "target_left",
                    "target_right",
                    "target_top",
                    "target_bottom",
                    "d_object_h",
                    "d_object_w",
                    "x_object",
                    "y_object",
                    "z_object",
                    "d_object",
                    "x_error",
                    "y_error",
                    "delta_angle",
                    "camera_tilt_angle",
                    "delta_d_x",
                    "delta_d_z",
                    "g_vx_adjust",
                    "g_vy_adjust",
                    "g_vz_adjust",
                    "mav_veh_sys_stat_voltage_battery",
                    "mav_veh_sys_stat_current_battery",
                    "mav_veh_sys_stat_battery_remaining",
                    "g_mav_veh_rel_alt",
                    "mav_veh_gps_vx",
                    "mav_veh_gps_vy",
                    "mav_veh_gps_vz",
                    "mav_veh_gps_hdg",
                    "mav_veh_roll",
                    "mav_veh_pitch",
                    "mav_veh_yaw",
                    "mav_veh_rollspeed",
                    "mav_veh_pitchspeed",
                    "mav_veh_yawspeed",
                    "mav_veh_imu_ax",
                    "mav_veh_imu_ay",
                    "mav_veh_imu_az",
                    "mav_veh_imu_xgyro",
                    "mav_veh_imu_ygyro",
                    "mav_veh_imu_zgyro",
                    "g_mav_veh_rngfdr_current_distance",
                    "mav_veh_rngfdr_signal_quality",
                    "mav_veh_flow_comp_m_x",
                    "mav_veh_flow_comp_m_y",
                    "mav_veh_flow_x",
                    "mav_veh_flow_y",
                    "mav_veh_flow_quality",
                    "mav_veh_flow_rate_x",
                    "mav_veh_flow_rate_y",
                    "mav_veh_local_ned_x",
                    "mav_veh_local_ned_y",
                    "mav_veh_local_ned_z",
                    "mav_veh_local_ned_vx",
                    "mav_veh_local_ned_vy",
                    "mav_veh_local_ned_vz",
                    "mav_veh_q1_actual",
                    "mav_veh_q2_actual",
                    "mav_veh_q3_actual",
                    "mav_veh_q4_actual",
                    "mav_veh_roll_rate_actual",
                    "mav_veh_pitch_rate_actual",
                    "mav_veh_yaw_rate_actual",
                    "mav_veh_repr_offset_q[0]",
                    "mav_veh_repr_offset_q[1]",
                    "mav_veh_repr_offset_q[2]",
                    "mav_veh_repr_offset_q[3]",
                    "mav_veh_type",
                    "mav_veh_autopilot_type",
                    "mav_veh_base_mode",
                    " mav_veh_custom_mode",
                    "mav_veh_state",
                    "mav_veh_mavlink_version"});

    save_to_csv(file_name, data);
}

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
 * Function: init
 * Description: Initialize data log variables and files.
 ********************************************************************************/
bool DataLogger::init(void)
{
#ifdef BLD_JETSON_B01

    data_file_path = "../data/";

#elif BLD_WIN

    data_file_path = "../../data/";

#else

#error "Please define a build platform."

#endif

    headings_written = false;
    data_file_name = "data";
    file_name = "";
    data = {};

    file_name = generate_unique_filename(data_file_name);
    write_headers();

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Log data.
 ********************************************************************************/
void DataLogger::loop(void)
{
    data.clear();

    data.push_back({{std::to_string(app_elapsed_time),
                     std::to_string(system_state),
                     std::to_string(g_target_too_close),
                     std::to_string(g_target_valid),
                     std::to_string(target_detection_ID),
                     std::to_string(target_track_ID),
                     std::to_string(target_cntr_offset_x),
                     std::to_string(target_cntr_offset_y),
                     std::to_string(target_height),
                     std::to_string(target_width),
                     std::to_string(target_aspect),
                     std::to_string(target_left),
                     std::to_string(target_right),
                     std::to_string(target_top),
                     std::to_string(target_bottom),
                     std::to_string(d_object_h),
                     std::to_string(d_object_w),
                     std::to_string(x_object),
                     std::to_string(y_object),
                     std::to_string(z_object),
                     std::to_string(d_object),
                     std::to_string(x_error),
                     std::to_string(y_error),
                     std::to_string(delta_angle),
                     std::to_string(camera_tilt_angle),
                     std::to_string(delta_d_x),
                     std::to_string(delta_d_z),
                     std::to_string(g_vx_adjust),
                     std::to_string(g_vy_adjust),
                     std::to_string(g_vz_adjust),
                     std::to_string(mav_veh_sys_stat_voltage_battery),
                     std::to_string(mav_veh_sys_stat_current_battery),
                     std::to_string(mav_veh_sys_stat_battery_remaining),
                     std::to_string(g_mav_veh_rel_alt),
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
                     std::to_string(g_mav_veh_rngfdr_current_distance),
                     std::to_string(mav_veh_rngfdr_signal_quality),
                     std::to_string(mav_veh_flow_comp_m_x),
                     std::to_string(mav_veh_flow_comp_m_y),
                     std::to_string(mav_veh_flow_x),
                     std::to_string(mav_veh_flow_y),
                     std::to_string(mav_veh_flow_quality),
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
                     std::to_string(mav_veh_repr_offset_q[3]),
                     std::to_string(mav_veh_type),
                     std::to_string(mav_veh_autopilot_type),
                     std::to_string(mav_veh_base_mode),
                     std::to_string(mav_veh_custom_mode),
                     std::to_string(mav_veh_state),
                     std::to_string(mav_veh_mavlink_version)}});

    save_to_csv(file_name, data);
}
