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
#include "common_inc.h"
#include "datalog.h"
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include "mav_data_hub.h"
#include "path_planner.h"
#include "track_target.h"
#include "localize_target.h"
#include "detect_target.h"
#include "time_calc.h"
#include "system_controller.h"
#include "video_io.h"
#include <spdlog/spdlog.h>

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

DataLogger data_logger;
std::unique_ptr<MCAPLogger> DataLogger::mMCAPLogger = nullptr;
std::mutex DataLogger::s_mtx;

/********************************************************************************
 * Calibration definitions
 ********************************************************************************/

/********************************************************************************
 * Function definitions
 ********************************************************************************/
void write_headers(void);
void log_data(void);
void save_to_csv(const std::string &filename, const std::vector<std::vector<std::string>> &data);
std::string generate_unique_filename(const std::string &filename);
void save_params(const std::string &filename);

/********************************************************************************
 * Function: generate_unique_filename
 * Description: If file exists, append an incremented number to the file name.
 ********************************************************************************/
std::string generate_unique_filename(const std::string &filename, const std::string ext)
{
    int counter = 1;
    std::string new_file_name = data_file_path + filename + ext; // Add .csv extension initially
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

#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

/********************************************************************************
 * Function: mkColor
 * Description: Color for detected object annotations.
 ********************************************************************************/
static inline foxglove::Color mkColor(double r, double g, double b, double a = 1.0)
{
    foxglove::Color c;
    c.set_r(r);
    c.set_g(g);
    c.set_b(b);
    c.set_a(a);
    return c;
}

/********************************************************************************
 * Function: publishAnnotations
 * Description: Color for detected object annotations.
 ********************************************************************************/
void DataLogger::publishAnnotations(uint64_t ts_ns, const os::logger::Objects &objs)
{
    foxglove::ImageAnnotations anno; // <-- FIXED

    for (int i = 0; i < objs.detections_size(); ++i)
    {
        const auto &det = objs.detections(i);
        const auto &b = det.bbox();

        const double w = b.bbox_width_px();
        const double h = b.bbox_height_px();
        const double cx = b.bbox_center_x_px();
        const double cy = b.bbox_center_y_px();

        const double x0 = cx - 0.5 * w;
        const double y0 = cy - 0.5 * h;
        const double x1 = x0 + w;
        const double y1 = y0 + h;

        auto *poly = anno.add_points();
        poly->set_type(foxglove::LINE_LOOP);

        auto *ts = poly->mutable_timestamp();
        ts->set_seconds(static_cast<int64_t>(ts_ns / 1'000'000'000ULL));
        ts->set_nanos(static_cast<int32_t>(ts_ns % 1'000'000'000ULL));

        auto addPt = [&](double xx, double yy)
        {
            auto *p = poly->add_points();
            p->set_x(xx);
            p->set_y(yy);
        };

        addPt(x0, y0);
        addPt(x1, y0);
        addPt(x1, y1);
        addPt(x0, y1);

        poly->mutable_outline_colors()->CopyFrom(mkColor(0.1, 0.8, 0.2));
        poly->set_thickness(2.0);

        // Text
        auto *txt = anno.add_texts();
        txt->mutable_timestamp()->CopyFrom(*ts);
        txt->mutable_position()->set_x(x0);
        txt->mutable_position()->set_y(std::max(0.0, y0 - 2.0));

        std::stringstream ss;
        ss << "id:" << det.track_id()
           << " C:" << det.class_id()
           << " " << std::round(det.confidence() * 100.0) / 100.0;

        txt->set_text(ss.str());
        txt->set_font_size(14);
        txt->mutable_text_color()->CopyFrom(mkColor(1, 1, 1));
        txt->mutable_background_color()->CopyFrom(mkColor(0, 0, 0, 0.6));
    }

    // publish("/detection/annotations", anno, ts_ns);
}

/********************************************************************************
 * Function: log_data
 * Description: Write data to mcap file.
 ********************************************************************************/
void DataLogger::log_data(void)
{
    if (!mMCAPLogger)
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

        m.set_roll(g_mav_veh_roll_rad);
        m.set_pitch(g_mav_veh_pitch_rad);
        m.set_yaw(g_mav_veh_yaw_rad);
        m.set_rollspeed(g_mav_veh_roll_rate);
        m.set_pitchspeed(g_mav_veh_pitch_rate);
        m.set_yawspeed(g_mav_veh_yaw_rate);

        m.set_local_ned_x(g_mav_veh_pos_ned_x * (float)180.0 / M_PI);
        m.set_local_ned_y(g_mav_veh_pos_ned_y * (float)180.0 / M_PI);
        m.set_local_ned_z(g_mav_veh_pos_ned_z * (float)180.0 / M_PI);
        m.set_local_ned_vx(g_mav_veh_vel_ned_x * (float)180.0 / M_PI);
        m.set_local_ned_vy(g_mav_veh_vel_ned_y * (float)180.0 / M_PI);
        m.set_local_ned_vz(g_mav_veh_vel_ned_z * (float)180.0 / M_PI);

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
        os::logger::MavImuRaw m;
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

        m.set_current_distance(g_mav_rngfndr_dist_m);
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
        m.set_delta_angle(g_cam0_delta_angle_rad * (float)180.0f / M_PI);
        m.set_camera_tilt_angle(g_cam0_angle_rad * (float)180.0f / M_PI);

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
        m.set_cmd_yawrate_rps(g_ctrl_yaw_cmd);

        publish("/control/output", m, g_app_epoch_ns);
    }
}

#else

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
    datalog_record.push_back({"g_app_time_s",
                              "g_system_state",
                              "g_tgt_too_close",
                              "g_tgt_valid",
                              "g_tgt_detect_id",
                              "g_tgt_track_id",
                              "g_tgt_class_id",
                              "g_tgt_conf",
                              "g_tgt_cntr_offset_y_pix",
                              "g_tgt_cntr_offset_x_pix",
                              "g_tgt_cntr_offset_x_pix_filt,"
                              "g_tgt_cntr_offset_y_pix_filt,"
                              "g_tgt_height_pix",
                              "g_tgt_width_pix",
                              "g_tgt_aspect_ratio",
                              "g_tgt_left_px",
                              "g_tgt_right_px",
                              "g_tgt_top_px",
                              "g_tgt_bottom_px",
                              "g_tgt_los_dist_from_pix_height",
                              "g_tgt_los_dist_from_pix_width",
                              "g_tgt_pos_x_meas",
                              "g_tgt_pos_y_meas",
                              "g_tgt_pos_z_meas",
                              "g_tgt_pos_x_est",
                              "g_tgt_pos_y_est",
                              "g_tgt_vel_x_est",
                              "g_tgt_vel_y_est",
                              "g_tgt_acc_x_est",
                              "g_tgt_acc_y_est",
                              "g_tgt_meas_valid",
                              "g_tgt_los_dist_meas",
                              "g_pos_err_x",
                              "g_pos_err_y",
                              "g_cam0_delta_angle_rad",
                              "g_cam0_angle_rad",
                              "g_tgt_pos_x_delta",
                              "g_tgt_pos_z_delta",
                              "g_ctrl_vel_x_cmd",
                              "g_ctrl_vel_y_cmd",
                              "g_ctrl_vel_z_cmd",
                              "g_ctrl_yaw_tgt",
                              "g_yaw_err",
                              "g_mav_veh_yaw_adjusted",
                              "g_mav_batt_voltage_mv",
                              "g_mav_batt_current_ma",
                              "g_mav_batt_remaining_pct",
                              "g_mav_gps_alt_rel",
                              "g_mav_gps_vel_x",
                              "g_mav_gps_vel_y",
                              "g_mav_gps_vel_z",
                              "g_mav_gps_heading_cdeg",
                              "g_mav_veh_roll_rad",
                              "g_mav_veh_pitch_rad",
                              "g_mav_veh_yaw_rad",
                              "g_mav_veh_roll_rate",
                              "g_mav_veh_pitch_rate",
                              "g_mav_veh_yaw_rate",
                              "g_mav_imu_accel_x",
                              "g_mav_imu_accel_y",
                              "g_mav_imu_accel_z",
                              "g_mav_imu_gyro_x",
                              "g_mav_imu_gyro_y",
                              "g_mav_imu_gyro_z",
                              "g_mav_rngfndr_dist_m",
                              "g_mav_rngfndr_quality",
                              "g_mav_flow_vel_x",
                              "g_mav_flow_vel_y",
                              "g_mav_flow_px_x",
                              "g_mav_flow_px_y",
                              "g_mav_flow_quality",
                              "g_mav_flow_rate_x",
                              "g_mav_flow_rate_y",
                              "g_mav_veh_pos_ned_x",
                              "g_mav_veh_pos_ned_y",
                              "g_mav_veh_pos_ned_z",
                              "g_mav_veh_vel_ned_x",
                              "g_mav_veh_vel_ned_y",
                              "g_mav_veh_vel_ned_z",
                              "g_mav_att_actual_q1",
                              "g_mav_att_actual_q2",
                              "g_mav_att_actual_q3",
                              "g_mav_att_actual_q4",
                              "g_mav_att_actual_roll_rate",
                              "g_mav_att_actual_pitch_rate",
                              "g_mav_att_actual_yaw_rate",
                              "g_mav_att_repr_offset_q[0]",
                              "g_mav_att_repr_offset_q[1]",
                              "g_mav_att_repr_offset_q[2]",
                              "g_mav_att_repr_offset_q[3]",
                              "g_mav_type",
                              "g_mav_autopilot_type",
                              "g_mav_mode_base",
                              "g_mav_mode_custom",
                              "g_mav_state",
                              "g_mav_version"});

    save_to_csv(unique_file_name, datalog_record);
}

/********************************************************************************
 * Function: log_data
 * Description: Log current signals into csv file.
 ********************************************************************************/
void log_data(void)
{
    datalog_record.clear();

    datalog_record.push_back({{std::to_string(g_app_time_s),
                               std::to_string(g_system_state),
                               std::to_string(g_tgt_too_close),
                               std::to_string(g_tgt_valid),
                               std::to_string(g_tgt_detect_id),
                               std::to_string(g_tgt_track_id),
                               std::to_string(g_tgt_class_id),
                               std::to_string(g_tgt_conf),
                               std::to_string(g_tgt_cntr_offset_y_pix),
                               std::to_string(g_tgt_cntr_offset_x_pix),
                               std::to_string(g_tgt_cntr_offset_x_pix_filt),
                               std::to_string(g_tgt_cntr_offset_y_pix_filt),
                               std::to_string(g_tgt_height_pix),
                               std::to_string(g_tgt_width_pix),
                               std::to_string(g_tgt_aspect_ratio),
                               std::to_string(g_tgt_left_px),
                               std::to_string(g_tgt_right_px),
                               std::to_string(g_tgt_top_px),
                               std::to_string(g_tgt_bottom_px),
                               std::to_string(g_tgt_los_dist_from_pix_height),
                               std::to_string(g_tgt_los_dist_from_pix_width),
                               std::to_string(g_tgt_pos_x_meas),
                               std::to_string(g_tgt_pos_y_meas),
                               std::to_string(g_tgt_pos_z_meas),
                               std::to_string(g_tgt_pos_x_est),
                               std::to_string(g_tgt_pos_y_est),
                               std::to_string(g_tgt_vel_x_est),
                               std::to_string(g_tgt_vel_y_est),
                               std::to_string(g_tgt_acc_x_est),
                               std::to_string(g_tgt_acc_y_est),
                               std::to_string(g_tgt_meas_valid),
                               std::to_string(g_tgt_los_dist_meas),
                               std::to_string(g_pos_err_x),
                               std::to_string(g_pos_err_y),
                               std::to_string(g_cam0_delta_angle_rad),
                               std::to_string(g_cam0_angle_rad),
                               std::to_string(g_tgt_pos_x_delta),
                               std::to_string(g_tgt_pos_z_delta),
                               std::to_string(g_ctrl_vel_x_cmd),
                               std::to_string(g_ctrl_vel_y_cmd),
                               std::to_string(g_ctrl_vel_z_cmd),
                               std::to_string(g_ctrl_yaw_tgt),
                               std::to_string(g_yaw_err),
                               std::to_string(g_mav_veh_yaw_adjusted),
                               std::to_string(g_mav_batt_voltage_mv),
                               std::to_string(g_mav_batt_current_ma),
                               std::to_string(g_mav_batt_remaining_pct),
                               std::to_string(g_mav_gps_alt_rel),
                               std::to_string(g_mav_gps_vel_x),
                               std::to_string(g_mav_gps_vel_y),
                               std::to_string(g_mav_gps_vel_z),
                               std::to_string(g_mav_gps_heading_cdeg),
                               std::to_string(g_mav_veh_roll_rad),
                               std::to_string(g_mav_veh_pitch_rad),
                               std::to_string(g_mav_veh_yaw_rad),
                               std::to_string(g_mav_veh_roll_rate),
                               std::to_string(g_mav_veh_pitch_rate),
                               std::to_string(g_mav_veh_yaw_rate),
                               std::to_string(g_mav_imu_accel_x),
                               std::to_string(g_mav_imu_accel_y),
                               std::to_string(g_mav_imu_accel_z),
                               std::to_string(g_mav_imu_gyro_x),
                               std::to_string(g_mav_imu_gyro_y),
                               std::to_string(g_mav_imu_gyro_z),
                               std::to_string(g_mav_rngfndr_dist_m),
                               std::to_string(g_mav_rngfndr_quality),
                               std::to_string(g_mav_flow_vel_x),
                               std::to_string(g_mav_flow_vel_y),
                               std::to_string(g_mav_flow_px_x),
                               std::to_string(g_mav_flow_px_y),
                               std::to_string(g_mav_flow_quality),
                               std::to_string(g_mav_flow_rate_x),
                               std::to_string(g_mav_flow_rate_y),
                               std::to_string(g_mav_veh_pos_ned_x),
                               std::to_string(g_mav_veh_pos_ned_y),
                               std::to_string(g_mav_veh_pos_ned_z),
                               std::to_string(g_mav_veh_vel_ned_x),
                               std::to_string(g_mav_veh_vel_ned_y),
                               std::to_string(g_mav_veh_vel_ned_z),
                               std::to_string(g_mav_att_actual_q1),
                               std::to_string(g_mav_att_actual_q2),
                               std::to_string(g_mav_att_actual_q3),
                               std::to_string(g_mav_att_actual_q4),
                               std::to_string(g_mav_att_actual_roll_rate),
                               std::to_string(g_mav_att_actual_pitch_rate),
                               std::to_string(g_mav_att_actual_yaw_rate),
                               std::to_string(g_mav_att_repr_offset_q[0]),
                               std::to_string(g_mav_att_repr_offset_q[1]),
                               std::to_string(g_mav_att_repr_offset_q[2]),
                               std::to_string(g_mav_att_repr_offset_q[3]),
                               std::to_string(g_mav_type),
                               std::to_string(g_mav_autopilot_type),
                               std::to_string(g_mav_mode_base),
                               std::to_string(g_mav_mode_custom),
                               std::to_string(g_mav_state),
                               std::to_string(g_mav_version)}});

    save_to_csv(unique_file_name, datalog_record);
}

#endif // logging options

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

    data_file_name = "data";
    unique_file_name = "";

#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

    const std::string unique_file_name = generate_unique_filename(data_file_name, ".mcap");
    mMCAPLogger = std::make_unique<MCAPLogger>(unique_file_name, "profile");

    // Register channels (schema names must match package.Message)
    mMCAPLogger->addChannel("/system/state", "os.logger.SystemStateMsg", "protobuf");
    mMCAPLogger->addChannel("/target/detection", "os.logger.TargetDetection", "protobuf");
    mMCAPLogger->addChannel("/target/state", "os.logger.TargetDetection", "protobuf");
    mMCAPLogger->addChannel("/control/output", "os.logger.ControlOutput", "protobuf");

    mMCAPLogger->addChannel("/mav/system", "os.logger.MavSystem", "protobuf");
    mMCAPLogger->addChannel("/mav/kinematics", "os.logger.MavKinematics", "protobuf");
    mMCAPLogger->addChannel("/mav/imu", "os.logger.MavImuRaw", "protobuf");
    mMCAPLogger->addChannel("/mav/rangefinder", "os.logger.MavRangefinder", "protobuf");
    mMCAPLogger->addChannel("/mav/flow", "os.logger.MavOpticalFlow", "protobuf");

    mMCAPLogger->addChannel("/detection/objects", "os.logger.Objects", "protobuf");
    // mMCAPLogger->addChannel("/detection/annotations", "os.logger.ImageAnnotations", "protobuf");

#else

    headings_written = false;
    datalog_record = {};
    unique_file_name = generate_unique_filename(data_file_name, ".csv");
    write_headers();

#endif

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
    if (mMCAPLogger)
    {
        mMCAPLogger->close();
        mMCAPLogger.reset();
    }
}