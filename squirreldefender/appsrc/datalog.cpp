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
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>

#include <spdlog/spdlog.h>

#include "common_inc.h"
#include "datalog.h"
#include "mav_data_hub.h"
#include "velocity_controller.h"
#include "target_tracking.h"
#include "target_localization.h"
#include "target_detection.h"
#include "time_calc.h"
#include "system_controller.h"
#include "video_io.h"

/********************************************************************************
 * Typedefs
 ********************************************************************************/

/********************************************************************************
 * Private macros and defines
 ********************************************************************************/

/********************************************************************************
 * Object definitions
 ********************************************************************************/
std::string data_file_path;
std::string data_file_name;
std::string unique_file_name;
std::unique_ptr<MCAPLogger> DataLogger::mcap_logger = nullptr;
std::mutex DataLogger::s_mtx;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
std::string generate_unique_filename(const std::string &filename, const std::string ext);
void save_params(const std::string &filename);
void log_data(void);

/********************************************************************************
 * Function: generate_unique_filename
 * Description: If file exists, append an incremented number to the file name.
 ********************************************************************************/
std::string generate_unique_filename(const std::string &filename, const std::string ext)
{
    int counter = 1;
    std::string new_file_name = data_file_path + filename + ext;
    std::ifstream file(new_file_name);

    while (file.is_open() == true)
    {
        file.close();
        // If the file exists, append the counter number and try again
        new_file_name = data_file_path + filename + "_" + std::to_string(counter++) + ext;
        file.open(new_file_name);
    }

    return new_file_name;
}

/********************************************************************************
 * Function: save_params
 * Description: Copy params.json (one level above data folder)
 *              into the data folder with matching index suffix.
 ********************************************************************************/
void save_params(const std::string &mcap_filename)
{
    // Derive suffix (e.g., "_3" from "data_3.mcap")
    std::string number_suffix;
    const size_t underscore_pos = mcap_filename.find_last_of('_');
    const size_t dot_pos = mcap_filename.find_last_of('.');
    if (underscore_pos != std::string::npos &&
        dot_pos != std::string::npos &&
        dot_pos > underscore_pos)
    {
        number_suffix = mcap_filename.substr(underscore_pos, dot_pos - underscore_pos);
    }

    // --- Use the known path instead of parsing ---
    // data_file_path points to "../data/"
    // params.json is one level up from that ("../params.json")
    const std::string src_path = data_file_path.substr(0, data_file_path.find_last_of('/', data_file_path.length() - 2)) + "/params.json";
    const std::string dst_path = data_file_path + "params" + number_suffix + ".json";

    std::ifstream src(src_path);
    if (!src.is_open())
    {
        spdlog::error("⚠️  Could not open {} for reading.", src_path);
        return;
    }

    std::ofstream dst(dst_path);
    if (!dst.is_open())
    {
        spdlog::error("⚠️  Could not create {} for writing.", dst_path);
        src.close();
        return;
    }

    std::string line;
    while (std::getline(src, line))
        dst << line << '\n';

    src.close();
    dst.close();
}

/********************************************************************************
 * Function: log_data
 * Description: Write data to mcap file.
 ********************************************************************************/
void DataLogger::log_data(void)
{
    if (!mcap_logger)
    {
        return;
    }

    {
        os::logger::SystemStateMsg m;
        logTime(m.mutable_t(), g_app_epoch_ns);
        m.set_frame_id(g_cam0_frame_id);
        m.set_system_state(static_cast<int32_t>(g_system_state));
        m.set_app_elapsed_time_sec(g_app_time_s);
        publish("/system/state", m, g_app_epoch_ns);
    }

    // MAV system / metadata
    {
        os::logger::MavSystem m;
        logTime(m.mutable_t(), g_app_epoch_ns);
        m.set_frame_id(g_cam0_frame_id);

        m.set_sys_stat_voltage_battery(g_mav_batt_voltage_mv);
        m.set_sys_stat_current_battery(g_mav_batt_current_ma);
        m.set_sys_stat_battery_remaining(g_mav_batt_remaining_pct);
        m.set_rel_alt(g_mav_gps_alt_rel);

        m.set_veh_type(g_mav_type);
        m.set_autopilot_type(g_mav_autopilot_type);
        m.set_base_mode(g_mav_mode_base);
        m.set_custom_mode(g_mav_mode_custom);
        m.set_state(g_mav_state);
        m.set_mavlink_version(g_mav_version);

        publish("/mav/system", m, g_app_epoch_ns);
    }

    // MAV kinematics / attitude
    {
        os::logger::MavKinematics m;
        logTime(m.mutable_t(), g_app_epoch_ns);
        m.set_frame_id(g_cam0_frame_id);

        m.set_gps_vx(g_mav_gps_vel_x);
        m.set_gps_vy(g_mav_gps_vel_y);
        m.set_gps_vz(g_mav_gps_vel_z);
        m.set_gps_hdg(g_mav_gps_heading_cdeg);

        m.set_roll(g_mav_veh_roll_rad * (float)180.0 / M_PI);
        m.set_pitch(g_mav_veh_pitch_rad * (float)180.0 / M_PI);
        m.set_yaw(g_mav_veh_yaw_rad * (float)180.0 / M_PI);
        m.set_rollspeed(g_mav_veh_roll_rate * (float)180.0 / M_PI);
        m.set_pitchspeed(g_mav_veh_pitch_rate * (float)180.0 / M_PI);
        m.set_yawspeed(g_mav_veh_yaw_rate * (float)180.0 / M_PI);

        m.set_local_ned_x(g_mav_veh_pos_ned_x);
        m.set_local_ned_y(g_mav_veh_pos_ned_y);
        m.set_local_ned_z(g_mav_veh_pos_ned_z);
        m.set_local_ned_vx(g_mav_veh_vel_ned_x);
        m.set_local_ned_vy(g_mav_veh_vel_ned_y);
        m.set_local_ned_vz(g_mav_veh_vel_ned_z);

        m.set_q1_actual(g_mav_att_actual_q1);
        m.set_q2_actual(g_mav_att_actual_q2);
        m.set_q3_actual(g_mav_att_actual_q3);
        m.set_q4_actual(g_mav_att_actual_q4);

        m.set_roll_rate_actual(g_mav_att_actual_roll_rate * (float)180.0 / M_PI);
        m.set_pitch_rate_actual(g_mav_att_actual_pitch_rate * (float)180.0 / M_PI);
        m.set_yaw_rate_actual(g_mav_att_actual_yaw_rate * (float)180.0 / M_PI);

        for (int i = 0; i < 4; ++i)
            m.add_repr_offset_q(g_mav_att_repr_offset_q[i]);

        publish("/mav/kinematics", m, g_app_epoch_ns);
    }

    // MAV IMU raw
    {
        os::logger::MavImu m;
        logTime(m.mutable_t(), g_app_epoch_ns);
        m.set_frame_id(g_cam0_frame_id);

        m.set_imu_ax(g_mav_imu_accel_x);
        m.set_imu_ay(g_mav_imu_accel_y);
        m.set_imu_az(g_mav_imu_accel_z);
        m.set_imu_xgyro(g_mav_imu_gyro_x);
        m.set_imu_ygyro(g_mav_imu_gyro_y);
        m.set_imu_zgyro(g_mav_imu_gyro_z);

        publish("/mav/imu", m, g_app_epoch_ns);
    }

    // Rangefinder
    {
        os::logger::MavRangefinder m;
        logTime(m.mutable_t(), g_app_epoch_ns);
        m.set_frame_id(g_cam0_frame_id);

        m.set_current_distance(g_mav_rngfndr_dist_cm);
        m.set_signal_quality(g_mav_rngfndr_quality);

        publish("/mav/rangefinder", m, g_app_epoch_ns);
    }

    // Optical flow
    {
        os::logger::MavOpticalFlow m;
        logTime(m.mutable_t(), g_app_epoch_ns);
        m.set_frame_id(g_cam0_frame_id);

        m.set_flow_comp_m_x(g_mav_flow_vel_x);
        m.set_flow_comp_m_y(g_mav_flow_vel_y);

        m.set_flow_x(g_mav_flow_px_x);
        m.set_flow_y(g_mav_flow_px_y);
        m.set_flow_quality(g_mav_flow_quality);

        m.set_flow_rate_x(g_mav_flow_rate_x);
        m.set_flow_rate_y(g_mav_flow_rate_y);

        publish("/mav/flow", m, g_app_epoch_ns);
    }

    // Detected objects
    {
        os::logger::Objects m;
        logTime(m.mutable_time(), g_app_epoch_ns);
        m.set_frame_id(g_cam0_frame_id);

        m.mutable_detections()->Reserve(g_det_yolo_list.size());

        for (const auto &d : g_det_yolo_list)
        {
            auto *out = m.add_detections();
            out->set_track_id(0);
            out->set_class_id(d.label);
            out->set_confidence(d.probability);

            auto *b = out->mutable_bbox();
            b->set_bbox_center_x_px(d.rect.x + 0.5f * d.rect.width);
            b->set_bbox_center_y_px(d.rect.y + 0.5f * d.rect.height);
            b->set_bbox_height_px(d.rect.height);
            b->set_bbox_width_px(d.rect.width);
        }

        publish("/detection/objects", m, g_app_epoch_ns);
    }

    // Target detections
    {
        os::logger::TargetDetection m;
        logTime(m.mutable_t(), g_app_epoch_ns);
        m.set_frame_id(g_cam0_frame_id);

        // Detection validity + IDs
        m.set_is_target_detected(g_tgt_valid);
        m.set_detection_id(g_tgt_detect_id);
        m.set_track_id(g_tgt_track_id);
        m.set_class_id(g_tgt_class_id);
        m.set_confidence(g_tgt_conf);

        // Raw bbox geometry in pixels
        m.set_bbox_center_offset_x_px_raw(g_tgt_cntr_offset_x_pix);
        m.set_bbox_center_offset_y_px_raw(g_tgt_cntr_offset_y_pix);

        m.set_bbox_height_px(g_tgt_height_pix);
        m.set_bbox_width_px(g_tgt_width_pix);
        m.set_bbox_aspect_ratio(g_tgt_aspect_ratio);

        m.set_bbox_left_px(g_tgt_left_px);
        m.set_bbox_right_px(g_tgt_right_px);
        m.set_bbox_top_px(g_tgt_top_px);
        m.set_bbox_bottom_px(g_tgt_bottom_px);

        // Filtered pixel center offsets
        m.set_bbox_center_offset_x_px_filt(g_tgt_cntr_offset_x_pix_filt);
        m.set_bbox_center_offset_y_px_filt(g_tgt_cntr_offset_y_pix_filt);

        publish("/target/detection", m, g_app_epoch_ns);
    }

    // Target state
    {
        os::logger::TargetState m;
        logTime(m.mutable_t(), g_app_epoch_ns);
        m.set_frame_id(g_cam0_frame_id);

        // Camera and angular geometry (converted to degrees where appropriate)
        m.set_delta_angle(g_cam0_delta_angle_rad * (float)180.0 / M_PI);
        m.set_camera_tilt_angle(g_cam0_angle_rad * (float)180.0 / M_PI);

        m.set_delta_distance_x_m(g_tgt_pos_x_delta);
        m.set_delta_distance_z_m(g_tgt_pos_z_delta);

        // EKF estimated target state
        m.set_pos_x_est(g_tgt_pos_x_est);
        m.set_pos_y_est(g_tgt_pos_y_est);
        m.set_vel_x_est(g_tgt_vel_x_est);
        m.set_vel_y_est(g_tgt_vel_y_est);
        m.set_acc_x_est(g_tgt_acc_x_est);
        m.set_acc_y_est(g_tgt_acc_y_est);

        // Post-filter measurement usability
        m.set_is_measurement_valid(g_tgt_meas_valid);
        publish("/target/state", m, g_app_epoch_ns);
    }

    // Control adjustments
    {
        os::logger::ControlOutput m;
        logTime(m.mutable_t(), g_app_epoch_ns);
        m.set_frame_id(g_cam0_frame_id);

        m.set_is_target_too_close(g_tgt_too_close);

        m.set_pos_error_x_m(g_pos_err_x);
        m.set_pos_error_y_m(g_pos_err_y);
        m.set_yaw_error(g_yaw_err);
        m.set_yaw_playback(g_veh_yaw_playback_adj);
        m.set_yaw_target(g_ctrl_yaw_tgt);

        m.set_cmd_vel_x_mps(g_ctrl_vel_x_cmd);
        m.set_cmd_vel_y_mps(g_ctrl_vel_y_cmd);
        m.set_cmd_vel_z_mps(g_ctrl_vel_z_cmd);
        m.set_cmd_yaw_rad(g_ctrl_yaw_cmd);

        publish("/control/output", m, g_app_epoch_ns);
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
#if defined(BLD_JETSON_B01) || defined(BLD_JETSON_ORIN) || defined(BLD_WSL)

    data_file_path = "../data/";

#elif defined(BLD_WIN)

    data_file_path = "../../data/";

#else

#error "Please define a build platform."

#endif

    data_file_name = "data";
    unique_file_name = generate_unique_filename(data_file_name, ".mcap");
    mcap_logger = std::make_unique<MCAPLogger>(unique_file_name, "profile");

    // Register channels (schema names must match package.Message)
    mcap_logger->addChannel("/system/state", "os.logger.SystemStateMsg", "protobuf");
    mcap_logger->addChannel("/target/detection", "os.logger.TargetDetection", "protobuf");
    mcap_logger->addChannel("/target/state", "os.logger.TargetState", "protobuf");
    mcap_logger->addChannel("/control/output", "os.logger.ControlOutput", "protobuf");
    mcap_logger->addChannel("/mav/system", "os.logger.MavSystem", "protobuf");
    mcap_logger->addChannel("/mav/kinematics", "os.logger.MavKinematics", "protobuf");
    mcap_logger->addChannel("/mav/imu", "os.logger.MavImu", "protobuf");
    mcap_logger->addChannel("/mav/rangefinder", "os.logger.MavRangefinder", "protobuf");
    mcap_logger->addChannel("/mav/flow", "os.logger.MavOpticalFlow", "protobuf");
    mcap_logger->addChannel("/detection/objects", "os.logger.Objects", "protobuf");

    save_params(unique_file_name);

    return true;
}

/********************************************************************************
 * Function: loop
 * Description: Log data.
 ********************************************************************************/
void DataLogger::loop(void)
{
    log_data();
}

/********************************************************************************
 * Function: shutdown
 * Description: Clean up code to run before program exits.
 ********************************************************************************/
void DataLogger::shutdown(void)
{
    if (mcap_logger)
    {
        mcap_logger->close();
        mcap_logger.reset();
    }
}