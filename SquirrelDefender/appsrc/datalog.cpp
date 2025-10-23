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
    foxglove::ImageAnnotations anno;

    for (int i = 0; i < objs.detections_size(); ++i)
    {
        const auto &det = objs.detections(i);
        const auto &b = det.box();
        const double w = b.w(), h = b.h();
        const double x0 = b.cx() - 0.5 * w, y0 = b.cy() - 0.5 * h;
        const double x1 = x0 + w, y1 = y0 + h;

        auto *poly = anno.add_points();
        poly->set_type(foxglove::LINE_LOOP);
        auto *t = poly->mutable_timestamp();
        t->set_seconds(static_cast<int64_t>(ts_ns / 1000000000ULL));
        t->set_nanos(static_cast<int32_t>(ts_ns % 1000000000ULL));

        auto addPt = [&](double x, double y)
        { auto* p = poly->add_points(); p->set_x(x); p->set_y(y); };
        addPt(x0, y0);
        addPt(x1, y0);
        addPt(x1, y1);
        addPt(x0, y1);

        poly->mutable_outline_color()->CopyFrom(mkColor(0.1, 0.8, 0.2));
        poly->set_thickness(2.0);

        auto *txt = anno.add_texts();
        txt->mutable_timestamp()->CopyFrom(*t);
        txt->mutable_position()->set_x(x0);
        txt->mutable_position()->set_y(std::max(0.0, y0 - 2.0));
        txt->set_text("id:" + std::to_string(det.id()) +
                      " L:" + std::to_string(det.label()) +
                      " " + std::to_string(std::round(det.score() * 100) / 100.0));
        txt->set_font_size(14);
        txt->mutable_text_color()->CopyFrom(mkColor(1, 1, 1));
        txt->mutable_background_color()->CopyFrom(mkColor(0, 0, 0, 0.6));
    }
}

#if defined(BLD_JETSON_ORIN_NANO) || defined(BLD_WSL)

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

    // System state
    {
        os::logger::SystemStateMsg m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_system_state(static_cast<int32_t>(g_system_state));
        m.set_app_elapsed_time(g_app_elapsed_time);
        DataLogger::publish("/system/state", m, g_epoch_ns);
    }

    // MAV system / metadata
    {
        os::logger::MavSystem m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_frame_id(g_frame_id);
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
        DataLogger::publish("/mav/system", m, g_epoch_ns);
    }

    // MAV kinematics / attitude
    {
        os::logger::MavKinematics m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_frame_id(g_frame_id);
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
        DataLogger::publish("/mav/kinematics", m, g_epoch_ns);
    }

    // MAV IMU raw
    {
        os::logger::MavImuRaw m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_frame_id(g_frame_id);
        m.set_imu_ax(static_cast<int32_t>(g_mav_veh_imu_ax));
        m.set_imu_ay(static_cast<int32_t>(g_mav_veh_imu_ay));
        m.set_imu_az(static_cast<int32_t>(g_mav_veh_imu_az));
        m.set_imu_xgyro(static_cast<int32_t>(g_mav_veh_imu_xgyro));
        m.set_imu_ygyro(static_cast<int32_t>(g_mav_veh_imu_ygyro));
        m.set_imu_zgyro(static_cast<int32_t>(g_mav_veh_imu_zgyro));
        DataLogger::publish("/mav/imu", m, g_epoch_ns);
    }

    // Rangefinder
    {
        os::logger::MavRangefinder m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_frame_id(g_frame_id);
        m.set_current_distance(static_cast<uint32_t>(g_mav_veh_rngfdr_current_distance));
        m.set_signal_quality(static_cast<uint32_t>(g_mav_veh_rngfdr_signal_quality));
        DataLogger::publish("/mav/rangefinder", m, g_epoch_ns);
    }

    // Optical flow
    {
        os::logger::MavOpticalFlow m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_frame_id(g_frame_id);
        m.set_flow_comp_m_x(g_mav_veh_flow_comp_m_x);
        m.set_flow_comp_m_y(g_mav_veh_flow_comp_m_y);
        m.set_flow_x(static_cast<int32_t>(g_mav_veh_flow_x));
        m.set_flow_y(static_cast<int32_t>(g_mav_veh_flow_y));
        m.set_flow_quality(static_cast<uint32_t>(g_mav_veh_flow_quality));
        m.set_flow_rate_x(g_mav_veh_flow_rate_x);
        m.set_flow_rate_y(g_mav_veh_flow_rate_y);
        DataLogger::publish("/mav/flow", m, g_epoch_ns);
    }

    // Detected objects
    {
        os::logger::Objects m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_frame_id(g_frame_id);

        // Reserve to avoid reallocs
        m.mutable_detections()->Reserve(static_cast<int>(g_yolo_detections.size()));

        int32_t detectionCount = 0;

        for (const auto &d : g_yolo_detections)
        {
            auto *out = m.add_detections();
            out->set_id(detectionCount); // no track ID's yet
            out->set_label(static_cast<uint32_t>(d.label));
            out->set_score(d.probability);

            // Fill bbox (note: your proto is cx, cy, h=3, w=4)
            auto *b = out->mutable_box();

            // If you store center/size already:
            b->set_cx(d.rect.x + 0.5f * d.rect.width);
            b->set_cy(d.rect.y + 0.5f * d.rect.height);
            b->set_h(d.rect.height);
            b->set_w(d.rect.width);

            ++detectionCount;
        }

        // Same publish style you used for TargetLocation
        DataLogger::publish("/detection/objects", m, g_epoch_ns);
        publish("/camera/annotations", m, g_epoch_ns); // schema name must be foxglove.ImageAnnotations
    }

    // Target detections/tracks
    {
        os::logger::TargetDetection m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_frame_id(g_frame_id);
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
        DataLogger::publish("/target/detection", m, g_epoch_ns);
    }

    // Target estimate
    {
        os::logger::TargetLocation m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_frame_id(g_frame_id);
        m.set_d_target_h(g_d_target_h);
        m.set_d_target_w(g_d_target_w);
        m.set_x_target(g_x_target);
        m.set_y_target(g_y_target);
        m.set_z_target(g_z_target);
        m.set_d_target(g_d_target);
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
        DataLogger::publish("/target/location", m, g_epoch_ns);
    }

    // Control adjustments
    {
        os::logger::ControlOutput m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_frame_id(g_frame_id);
        m.set_target_too_close(g_target_too_close);
        m.set_x_error(g_x_error);
        m.set_y_error(g_y_error);
        m.set_yaw_error(g_yaw_target_error);
        m.set_yaw_adjusted_for_playback(g_mav_veh_yaw_adjusted_for_playback);
        m.set_yaw_target(g_yaw_target);
        m.set_vx_adjust(g_vx_adjust);
        m.set_vy_adjust(g_vy_adjust);
        m.set_vz_adjust(g_vz_adjust);
        m.set_yaw_adjust(g_yaw_adjust);
        DataLogger::publish("/path/control", m, g_epoch_ns);
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

    const std::string path = generate_unique_filename(data_file_name, ".mcap");
    mMCAPLogger = std::make_unique<MCAPLogger>(path, "profile");

    // Register channels (schema names must match package.Message)
    mMCAPLogger->addChannel("/system/state", "os.logger.SystemStateMsg", "protobuf");
    mMCAPLogger->addChannel("/target/detection", "os.logger.TargetDetection", "protobuf");
    mMCAPLogger->addChannel("/target/location", "os.logger.TargetLocation", "protobuf");
    mMCAPLogger->addChannel("/path/control", "os.logger.ControlOutput", "protobuf");
    mMCAPLogger->addChannel("/mav/system", "os.logger.MavSystem", "protobuf");
    mMCAPLogger->addChannel("/mav/kinematics", "os.logger.MavKinematics", "protobuf");
    mMCAPLogger->addChannel("/mav/imu", "os.logger.MavImuRaw", "protobuf");
    mMCAPLogger->addChannel("/mav/rangefinder", "os.logger.MavRangefinder", "protobuf");
    mMCAPLogger->addChannel("/mav/flow", "os.logger.MavOpticalFlow", "protobuf");
    mMCAPLogger->addChannel("/detection/objects", "os.logger.Objects", "protobuf");
    mMCAPLogger->addChannel("/detection/annotations", "foxglove.ImageAnnotations", "protobuf");

#else

    headings_written = false;
    datalog_record = {};
    unique_file_name = generate_unique_filename(data_file_name, ".csv");
    write_headers();

#endif

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