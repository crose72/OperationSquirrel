/********************************************************************************
 * @file    datalog.cpp
 * @author  Cameron Rose
 * @date    1/22/2025
 * @brief   Record key information in a text file for debugging and issue
 *          resolution.
 ********************************************************************************/

/********************************************************************************
 * Includes
 ********************************************************************************/
#include "datalog.h"
#include "Common.pb.h"
#include "TargetInfo.pb.h"
#include "Mavlink.pb.h"
#include "System.pb.h"
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
std::string unique_file_name;
std::vector<std::vector<std::string>> datalog_record;

DataLogger data_logger;
std::unique_ptr<MCAPLogger> DataLogger::mMCAPLogger = nullptr;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void write_headers(void);
void save_to_csv(const std::string &filename, const std::vector<std::vector<std::string>> &data);
std::string generate_unique_filename(const std::string &filename);
void log_data(void);

// Unique filename helper (your approach, targeting .mcap)
static std::string generate_unique_mcap_filename(const std::string &stem)
{
#if defined(BLD_JETSON_B01) || defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)
    const std::string baseDir = "../data/";
#elif defined(BLD_WIN)
    const std::string baseDir = "../../data/";
#else
#error "Please define a build platform."
#endif
    int counter = 0;
    while (true)
    {
        const std::string name = (counter == 0)
                                     ? (baseDir + stem + ".mcap")
                                     : (baseDir + stem + "_" + std::to_string(counter) + ".mcap");
        std::ifstream f(name, std::ios::binary);
        if (!f.good())
            return name;
        ++counter;
    }
}

static inline void fill_time(logger::Time *t, uint64_t ts_ns)
{
    t->set_timestamp_ns(ts_ns);
    t->set_timestamp_sec(static_cast<double>(ts_ns) * 1e-9);
}

// void DataLogger::logTargetInfo(float x, float y, uint64_t timestamp)
// {
//     logger::TargetInfo msg;
//     msg.set_x(x);
//     msg.set_y(y);
//     msg.set_timestamp(static_cast<double>(timestamp) * 1e-9);

//     std::string data;
//     bool success = msg.SerializeToString(&data);
//     if (!success)
//     {
//         std::cerr << "Serialization failed!" << std::endl;
//     }
//     else
//     {
//         std::cout << "yay" << std::endl;
//     }

//     // MCAP timestamp should be uint64_t nanoseconds, but can use whatever you want for now
//     // mcap_logger_.logMessage("/target_location", data, static_cast<uint64_t>(timestamp * 1e9));
//     bool ok = mMCAPLogger->logMessage("/target_info", data, timestamp);
//     if (!ok)
//     {
//         std::cerr << "MCAP logMessage failed!" << std::endl;
//     }
//     else
//     {
//         std::cout << "MCAP logMessage succeeded." << std::endl;
//     }
// }

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
    if (!datalog_record.empty())
    {
        size_t numRows = datalog_record.size();
        size_t numCols = datalog_record[0].size();

        for (size_t row = 0; row < numRows; ++row)
        {
            for (size_t col = 0; col < numCols; ++col)
            {
                outFile << datalog_record[row][col];
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
    datalog_record.push_back({"g_app_elapsed_time",
                              "g_system_state",
                              "g_target_too_close",
                              "g_target_valid",
                              "g_target_detection_id",
                              "g_target_track_id",
                              "g_detection_class",
                              "g_target_detection_conf",
                              "g_target_cntr_offset_x",
                              "g_target_cntr_offset_y",
                              "g_target_cntr_offset_x_filt,"
                              "g_target_cntr_offset_y_filt,"
                              "g_target_height",
                              "g_target_width",
                              "g_target_aspect",
                              "g_target_left",
                              "g_target_right",
                              "g_target_top",
                              "g_target_bottom",
                              "g_d_target_h",
                              "g_d_target_w",
                              "g_x_target",
                              "g_y_target",
                              "g_z_target",
                              "g_x_target_ekf",
                              "g_y_target_ekf",
                              "g_vx_target_ekf",
                              "g_vy_target_ekf",
                              "g_ax_target_ekf",
                              "g_ay_target_ekf",
                              "g_target_data_useful",
                              "g_d_target",
                              "g_x_error",
                              "g_y_error",
                              "g_delta_angle",
                              "g_camera_tilt_angle",
                              "g_delta_d_x",
                              "g_delta_d_z",
                              "g_vx_adjust",
                              "g_vy_adjust",
                              "g_vz_adjust",
                              "g_yaw_target",
                              "g_yaw_target_error",
                              "g_mav_veh_yaw_adjusted",
                              "g_mav_veh_sys_stat_voltage_battery",
                              "g_mav_veh_sys_stat_current_battery",
                              "g_mav_veh_sys_stat_battery_remaining",
                              "g_mav_veh_rel_alt",
                              "g_mav_veh_gps_vx",
                              "g_mav_veh_gps_vy",
                              "g_mav_veh_gps_vz",
                              "g_mav_veh_gps_hdg",
                              "g_mav_veh_roll",
                              "g_mav_veh_pitch",
                              "g_mav_veh_yaw",
                              "g_mav_veh_rollspeed",
                              "g_mav_veh_pitchspeed",
                              "g_mav_veh_yawspeed",
                              "g_mav_veh_imu_ax",
                              "g_mav_veh_imu_ay",
                              "g_mav_veh_imu_az",
                              "g_mav_veh_imu_xgyro",
                              "g_mav_veh_imu_ygyro",
                              "g_mav_veh_imu_zgyro",
                              "g_mav_veh_rngfdr_current_distance",
                              "g_mav_veh_rngfdr_signal_quality",
                              "g_mav_veh_flow_comp_m_x",
                              "g_mav_veh_flow_comp_m_y",
                              "g_mav_veh_flow_x",
                              "g_mav_veh_flow_y",
                              "g_mav_veh_flow_quality",
                              "g_mav_veh_flow_rate_x",
                              "g_mav_veh_flow_rate_y",
                              "g_mav_veh_local_ned_x",
                              "g_mav_veh_local_ned_y",
                              "g_mav_veh_local_ned_z",
                              "g_mav_veh_local_ned_vx",
                              "g_mav_veh_local_ned_vy",
                              "g_mav_veh_local_ned_vz",
                              "g_mav_veh_q1_actual",
                              "g_mav_veh_q2_actual",
                              "g_mav_veh_q3_actual",
                              "g_mav_veh_q4_actual",
                              "g_mav_veh_roll_rate_actual",
                              "g_mav_veh_pitch_rate_actual",
                              "g_mav_veh_yaw_rate_actual",
                              "g_mav_veh_repr_offset_q[0]",
                              "g_mav_veh_repr_offset_q[1]",
                              "g_mav_veh_repr_offset_q[2]",
                              "g_mav_veh_repr_offset_q[3]",
                              "g_mav_veh_type",
                              "g_mav_veh_autopilot_type",
                              "g_mav_veh_base_mode",
                              "g_mav_veh_custom_mode",
                              "g_mav_veh_state",
                              "g_mav_veh_mavlink_version"});

    save_to_csv(unique_file_name, datalog_record);
}

/********************************************************************************
 * Function: log_data
 * Description: Log current signals into csv file.
 ********************************************************************************/
void log_data(void)
{
    datalog_record.clear();

    datalog_record.push_back({{std::to_string(g_app_elapsed_time),
                               std::to_string(g_system_state),
                               std::to_string(g_target_too_close),
                               std::to_string(g_target_valid),
                               std::to_string(g_target_detection_id),
                               std::to_string(g_target_track_id),
                               std::to_string(g_detection_class),
                               std::to_string(g_target_detection_conf),
                               std::to_string(g_target_cntr_offset_x),
                               std::to_string(g_target_cntr_offset_y),
                               std::to_string(g_target_cntr_offset_x_filt),
                               std::to_string(g_target_cntr_offset_y_filt),
                               std::to_string(g_target_height),
                               std::to_string(g_target_width),
                               std::to_string(g_target_aspect),
                               std::to_string(g_target_left),
                               std::to_string(g_target_right),
                               std::to_string(g_target_top),
                               std::to_string(g_target_bottom),
                               std::to_string(g_d_target_h),
                               std::to_string(g_d_target_w),
                               std::to_string(g_x_target),
                               std::to_string(g_y_target),
                               std::to_string(g_z_target),
                               std::to_string(g_x_target_ekf),
                               std::to_string(g_y_target_ekf),
                               std::to_string(g_vx_target_ekf),
                               std::to_string(g_vy_target_ekf),
                               std::to_string(g_ax_target_ekf),
                               std::to_string(g_ay_target_ekf),
                               std::to_string(g_target_data_useful),
                               std::to_string(g_d_target),
                               std::to_string(g_x_error),
                               std::to_string(g_y_error),
                               std::to_string(g_delta_angle),
                               std::to_string(g_camera_tilt_angle),
                               std::to_string(g_delta_d_x),
                               std::to_string(g_delta_d_z),
                               std::to_string(g_vx_adjust),
                               std::to_string(g_vy_adjust),
                               std::to_string(g_vz_adjust),
                               std::to_string(g_yaw_target),
                               std::to_string(g_yaw_target_error),
                               std::to_string(g_mav_veh_yaw_adjusted),
                               std::to_string(g_mav_veh_sys_stat_voltage_battery),
                               std::to_string(g_mav_veh_sys_stat_current_battery),
                               std::to_string(g_mav_veh_sys_stat_battery_remaining),
                               std::to_string(g_mav_veh_rel_alt),
                               std::to_string(g_mav_veh_gps_vx),
                               std::to_string(g_mav_veh_gps_vy),
                               std::to_string(g_mav_veh_gps_vz),
                               std::to_string(g_mav_veh_gps_hdg),
                               std::to_string(g_mav_veh_roll),
                               std::to_string(g_mav_veh_pitch),
                               std::to_string(g_mav_veh_yaw),
                               std::to_string(g_mav_veh_rollspeed),
                               std::to_string(g_mav_veh_pitchspeed),
                               std::to_string(g_mav_veh_yawspeed),
                               std::to_string(g_mav_veh_imu_ax),
                               std::to_string(g_mav_veh_imu_ay),
                               std::to_string(g_mav_veh_imu_az),
                               std::to_string(g_mav_veh_imu_xgyro),
                               std::to_string(g_mav_veh_imu_ygyro),
                               std::to_string(g_mav_veh_imu_zgyro),
                               std::to_string(g_mav_veh_rngfdr_current_distance),
                               std::to_string(g_mav_veh_rngfdr_signal_quality),
                               std::to_string(g_mav_veh_flow_comp_m_x),
                               std::to_string(g_mav_veh_flow_comp_m_y),
                               std::to_string(g_mav_veh_flow_x),
                               std::to_string(g_mav_veh_flow_y),
                               std::to_string(g_mav_veh_flow_quality),
                               std::to_string(g_mav_veh_flow_rate_x),
                               std::to_string(g_mav_veh_flow_rate_y),
                               std::to_string(g_mav_veh_local_ned_x),
                               std::to_string(g_mav_veh_local_ned_y),
                               std::to_string(g_mav_veh_local_ned_z),
                               std::to_string(g_mav_veh_local_ned_vx),
                               std::to_string(g_mav_veh_local_ned_vy),
                               std::to_string(g_mav_veh_local_ned_vz),
                               std::to_string(g_mav_veh_q1_actual),
                               std::to_string(g_mav_veh_q2_actual),
                               std::to_string(g_mav_veh_q3_actual),
                               std::to_string(g_mav_veh_q4_actual),
                               std::to_string(g_mav_veh_roll_rate_actual),
                               std::to_string(g_mav_veh_pitch_rate_actual),
                               std::to_string(g_mav_veh_yaw_rate_actual),
                               std::to_string(g_mav_veh_repr_offset_q[0]),
                               std::to_string(g_mav_veh_repr_offset_q[1]),
                               std::to_string(g_mav_veh_repr_offset_q[2]),
                               std::to_string(g_mav_veh_repr_offset_q[3]),
                               std::to_string(g_mav_veh_type),
                               std::to_string(g_mav_veh_autopilot_type),
                               std::to_string(g_mav_veh_base_mode),
                               std::to_string(g_mav_veh_custom_mode),
                               std::to_string(g_mav_veh_state),
                               std::to_string(g_mav_veh_mavlink_version)}});

    save_to_csv(unique_file_name, datalog_record);
}

void DataLogger::log_data_mcap(void)
{
    if (!mMCAPLogger)
        return;
    const uint64_t ts_ns = g_epoch_ns;

    // --- System state ---
    {
        logger::SystemStateMsg m;
        fill_time(m.mutable_t(), ts_ns);
        m.set_system_state(static_cast<int32_t>(g_system_state));
        m.set_app_elapsed_time(g_app_elapsed_time);
        std::string bytes;
        if (m.SerializeToString(&bytes))
        {
            mMCAPLogger->logMessage("/system/state", bytes, ts_ns);
        }
    }

    // --- Target detections/tracks ---
    {
        logger::TargetDetectionTrack m;
        fill_time(m.mutable_t(), ts_ns);
        m.set_target_valid(g_target_valid);
        m.set_target_detection_id(static_cast<int32_t>(g_target_detection_id));
        m.set_target_track_id(static_cast<int32_t>(g_target_track_id));
        m.set_detection_class(g_detection_class);
        m.set_target_detection_conf(g_target_detection_conf);
        m.set_target_cntr_offset_x(g_target_cntr_offset_x);
        m.set_target_cntr_offset_y(g_target_cntr_offset_y);
        m.set_target_cntr_offset_x_filt(g_target_cntr_offset_x_filt);
        m.set_target_cntr_offset_y_filt(g_target_cntr_offset_y_filt);
        m.set_target_height(g_target_height);
        m.set_target_width(g_target_width);
        m.set_target_aspect(g_target_aspect);
        m.set_target_left(g_target_left);
        m.set_target_right(g_target_right);
        m.set_target_top(g_target_top);
        m.set_target_bottom(g_target_bottom);
        std::string bytes;
        if (m.SerializeToString(&bytes))
        {
            mMCAPLogger->logMessage("/target/detections", bytes, ts_ns);
        }
    }

    // --- Target estimate / EKF ---
    {
        logger::TargetEstimate m;
        fill_time(m.mutable_t(), ts_ns);
        m.set_d_target_h(g_d_target_h);
        m.set_d_target_w(g_d_target_w);
        m.set_x_target(g_x_target);
        m.set_y_target(g_y_target);
        m.set_z_target(g_z_target);
        m.set_d_target(g_d_target);
        m.set_x_error(g_x_error);
        m.set_y_error(g_y_error);
        m.set_delta_angle(g_delta_angle);
        m.set_camera_tilt_angle(g_camera_tilt_angle);
        m.set_delta_d_x(g_delta_d_x);
        m.set_delta_d_z(g_delta_d_z);
        m.set_x_target_ekf(g_x_target_ekf);
        m.set_y_target_ekf(g_y_target_ekf);
        m.set_vx_target_ekf(g_vx_target_ekf);
        m.set_vy_target_ekf(g_vy_target_ekf);
        m.set_ax_target_ekf(g_ax_target_ekf);
        m.set_ay_target_ekf(g_ay_target_ekf);
        m.set_target_data_useful(g_target_data_useful);
        std::string bytes;
        if (m.SerializeToString(&bytes))
        {
            mMCAPLogger->logMessage("/target/estimate", bytes, ts_ns);
        }
    }

    // --- Control adjustments ---
    {
        logger::ControlAdjust m;
        fill_time(m.mutable_t(), ts_ns);
        m.set_target_too_close(g_target_too_close);
        m.set_vx_adjust(g_vx_adjust);
        m.set_vy_adjust(g_vy_adjust);
        m.set_vz_adjust(g_vz_adjust);
        m.set_yaw_target(g_yaw_target);
        m.set_yaw_target_error(g_yaw_target_error);
        m.set_mav_veh_yaw_adjusted(g_mav_veh_yaw_adjusted);
        std::string bytes;
        if (m.SerializeToString(&bytes))
        {
            mMCAPLogger->logMessage("/control/adjust", bytes, ts_ns);
        }
    }

    // --- MAV system / metadata ---
    {
        logger::MavSystem m;
        fill_time(m.mutable_t(), ts_ns);
        m.set_sys_stat_voltage_battery(static_cast<uint32_t>(g_mav_veh_sys_stat_voltage_battery));
        m.set_sys_stat_current_battery(static_cast<int32_t>(g_mav_veh_sys_stat_current_battery));
        m.set_sys_stat_battery_remaining(static_cast<int32_t>(g_mav_veh_sys_stat_battery_remaining));
        m.set_rel_alt(static_cast<int32_t>(g_mav_veh_rel_alt));
        m.set_veh_type(static_cast<uint32_t>(g_mav_veh_type));
        m.set_autopilot_type(static_cast<uint32_t>(g_mav_veh_autopilot_type));
        m.set_base_mode(static_cast<uint32_t>(g_mav_veh_base_mode));
        m.set_custom_mode(static_cast<uint32_t>(g_mav_veh_custom_mode));
        m.set_state(static_cast<uint32_t>(g_mav_veh_state));
        m.set_mavlink_version(static_cast<uint32_t>(g_mav_veh_mavlink_version));
        std::string bytes;
        if (m.SerializeToString(&bytes))
        {
            mMCAPLogger->logMessage("/mav/system", bytes, ts_ns);
        }
    }

    // --- MAV kinematics / attitude ---
    {
        logger::MavKinematics m;
        fill_time(m.mutable_t(), ts_ns);
        m.set_gps_vx(static_cast<int32_t>(g_mav_veh_gps_vx));
        m.set_gps_vy(static_cast<int32_t>(g_mav_veh_gps_vy));
        m.set_gps_vz(static_cast<int32_t>(g_mav_veh_gps_vz));
        m.set_gps_hdg(static_cast<uint32_t>(g_mav_veh_gps_hdg));
        m.set_roll(g_mav_veh_roll);
        m.set_pitch(g_mav_veh_pitch);
        m.set_yaw(g_mav_veh_yaw);
        m.set_rollspeed(g_mav_veh_rollspeed);
        m.set_pitchspeed(g_mav_veh_pitchspeed);
        m.set_yawspeed(g_mav_veh_yawspeed);
        m.set_local_ned_x(g_mav_veh_local_ned_x);
        m.set_local_ned_y(g_mav_veh_local_ned_y);
        m.set_local_ned_z(g_mav_veh_local_ned_z);
        m.set_local_ned_vx(g_mav_veh_local_ned_vx);
        m.set_local_ned_vy(g_mav_veh_local_ned_vy);
        m.set_local_ned_vz(g_mav_veh_local_ned_vz);
        m.set_q1_actual(g_mav_veh_q1_actual);
        m.set_q2_actual(g_mav_veh_q2_actual);
        m.set_q3_actual(g_mav_veh_q3_actual);
        m.set_q4_actual(g_mav_veh_q4_actual);
        m.set_roll_rate_actual(g_mav_veh_roll_rate_actual);
        m.set_pitch_rate_actual(g_mav_veh_pitch_rate_actual);
        m.set_yaw_rate_actual(g_mav_veh_yaw_rate_actual);
        for (int i = 0; i < 4; ++i)
            m.add_repr_offset_q(g_mav_veh_repr_offset_q[i]);
        std::string bytes;
        if (m.SerializeToString(&bytes))
        {
            mMCAPLogger->logMessage("/mav/kinematics", bytes, ts_ns);
        }
    }

    // --- MAV IMU raw ---
    {
        logger::MavImuRaw m;
        fill_time(m.mutable_t(), ts_ns);
        m.set_imu_ax(static_cast<int32_t>(g_mav_veh_imu_ax));
        m.set_imu_ay(static_cast<int32_t>(g_mav_veh_imu_ay));
        m.set_imu_az(static_cast<int32_t>(g_mav_veh_imu_az));
        m.set_imu_xgyro(static_cast<int32_t>(g_mav_veh_imu_xgyro));
        m.set_imu_ygyro(static_cast<int32_t>(g_mav_veh_imu_ygyro));
        m.set_imu_zgyro(static_cast<int32_t>(g_mav_veh_imu_zgyro));
        std::string bytes;
        if (m.SerializeToString(&bytes))
        {
            mMCAPLogger->logMessage("/mav/imu_raw", bytes, ts_ns);
        }
    }

    // --- Rangefinder ---
    {
        logger::MavRangefinder m;
        fill_time(m.mutable_t(), ts_ns);
        m.set_current_distance(static_cast<uint32_t>(g_mav_veh_rngfdr_current_distance));
        m.set_signal_quality(static_cast<uint32_t>(g_mav_veh_rngfdr_signal_quality));
        std::string bytes;
        if (m.SerializeToString(&bytes))
        {
            mMCAPLogger->logMessage("/mav/rangefinder", bytes, ts_ns);
        }
    }

    // --- Optical flow ---
    {
        logger::MavOpticalFlow m;
        fill_time(m.mutable_t(), ts_ns);
        m.set_flow_comp_m_x(g_mav_veh_flow_comp_m_x);
        m.set_flow_comp_m_y(g_mav_veh_flow_comp_m_y);
        m.set_flow_x(static_cast<int32_t>(g_mav_veh_flow_x));
        m.set_flow_y(static_cast<int32_t>(g_mav_veh_flow_y));
        m.set_flow_quality(static_cast<uint32_t>(g_mav_veh_flow_quality));
        m.set_flow_rate_x(g_mav_veh_flow_rate_x);
        m.set_flow_rate_y(g_mav_veh_flow_rate_y);
        std::string bytes;
        if (m.SerializeToString(&bytes))
        {
            mMCAPLogger->logMessage("/mav/flow", bytes, ts_ns);
        }
    }
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
#if defined(BLD_JETSON_B01) || defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    data_file_path = "../data/";

#elif defined(BLD_WIN)

    data_file_path = "../../data/";

#else

#error "Please define a build platform."

#endif

    headings_written = false;
    data_file_name = "data";
    unique_file_name = "";
    datalog_record = {};

    const std::string path = generate_unique_mcap_filename(data_file_name);
    mMCAPLogger = std::make_unique<MCAPLogger>(path, "profile");

    // Register channels (schema names must match package.Message)
    mMCAPLogger->addChannel("/system/state", "logger.SystemStateMsg", "protobuf");
    mMCAPLogger->addChannel("/target/detections", "logger.TargetDetectionTrack", "protobuf");
    mMCAPLogger->addChannel("/target/estimate", "logger.TargetEstimate", "protobuf");
    mMCAPLogger->addChannel("/control/adjust", "logger.ControlAdjust", "protobuf");
    mMCAPLogger->addChannel("/mav/system", "logger.MavSystem", "protobuf");
    mMCAPLogger->addChannel("/mav/kinematics", "logger.MavKinematics", "protobuf");
    mMCAPLogger->addChannel("/mav/imu_raw", "logger.MavImuRaw", "protobuf");
    mMCAPLogger->addChannel("/mav/rangefinder", "logger.MavRangefinder", "protobuf");
    mMCAPLogger->addChannel("/mav/flow", "logger.MavOpticalFlow", "protobuf");

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Log data.
 ********************************************************************************/
void DataLogger::loop(void)
{
    static uint32_t dbg = 0;
    if ((dbg++ % 60) == 0)
    {
        std::cerr << "[LOG] ts=" << g_epoch_ns
                  << " roll=" << g_mav_veh_roll
                  << " tgt_valid=" << g_target_valid
                  << " x=" << g_x_target << "\n";
    }
    log_data_mcap();
}

/********************************************************************************
 * Function: shutdown
 * Description: Clean up code to run before program exits.
 ********************************************************************************/
void DataLogger::shutdown(void)
{
    if (mMCAPLogger)
    {
        mMCAPLogger->close();
        mMCAPLogger.reset();
    }
}