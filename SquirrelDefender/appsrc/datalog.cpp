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
void save_to_csv(const std::string &filename, const std::vector<std::vector<std::string>> &data);
std::string generate_unique_filename(const std::string &filename);
void log_data(void);

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

static inline foxglove::Color mkColor(double r, double g, double b, double a = 1.0)
{
    foxglove::Color c;
    c.set_r(r);
    c.set_g(g);
    c.set_b(b);
    c.set_a(a);
    return c;
}

void DataLogger::publishAnnotations(uint64_t ts_ns, const logger::Objects &objs)
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

void DataLogger::log_data(void)
{
    if (!mMCAPLogger)
    {
        return;
    }

    // --- System state ---
    {
        logger::SystemStateMsg m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_system_state(static_cast<int32_t>(g_system_state));
        m.set_app_elapsed_time(g_app_elapsed_time);
        DataLogger::publish("/system/state", m, g_epoch_ns);
    }

    // --- Target detections/tracks ---
    {
        logger::TargetDetectionTrack m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
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
        DataLogger::publish("/target/detections", m, g_epoch_ns);
    }

    // --- Control adjustments ---
    {
        logger::ControlAdjust m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_target_too_close(g_target_too_close);
        m.set_vx_adjust(g_vx_adjust);
        m.set_vy_adjust(g_vy_adjust);
        m.set_vz_adjust(g_vz_adjust);
        m.set_yaw_target(g_yaw_target);
        m.set_yaw_target_error(g_yaw_target_error);
        m.set_mav_veh_yaw_adjusted(g_mav_veh_yaw_adjusted);
        DataLogger::publish("/control/adjust", m, g_epoch_ns);
    }

    // --- MAV system / metadata ---
    {
        logger::MavSystem m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
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

    // --- MAV kinematics / attitude ---
    {
        logger::MavKinematics m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
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

    // --- MAV IMU raw ---
    {
        logger::MavImuRaw m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_imu_ax(static_cast<int32_t>(g_mav_veh_imu_ax));
        m.set_imu_ay(static_cast<int32_t>(g_mav_veh_imu_ay));
        m.set_imu_az(static_cast<int32_t>(g_mav_veh_imu_az));
        m.set_imu_xgyro(static_cast<int32_t>(g_mav_veh_imu_xgyro));
        m.set_imu_ygyro(static_cast<int32_t>(g_mav_veh_imu_ygyro));
        m.set_imu_zgyro(static_cast<int32_t>(g_mav_veh_imu_zgyro));
        DataLogger::publish("/mav/imu_raw", m, g_epoch_ns);
    }

    // --- Rangefinder ---
    {
        logger::MavRangefinder m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_current_distance(static_cast<uint32_t>(g_mav_veh_rngfdr_current_distance));
        m.set_signal_quality(static_cast<uint32_t>(g_mav_veh_rngfdr_signal_quality));
        DataLogger::publish("/mav/rangefinder", m, g_epoch_ns);
    }

    // --- Optical flow ---
    {
        logger::MavOpticalFlow m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
        m.set_flow_comp_m_x(g_mav_veh_flow_comp_m_x);
        m.set_flow_comp_m_y(g_mav_veh_flow_comp_m_y);
        m.set_flow_x(static_cast<int32_t>(g_mav_veh_flow_x));
        m.set_flow_y(static_cast<int32_t>(g_mav_veh_flow_y));
        m.set_flow_quality(static_cast<uint32_t>(g_mav_veh_flow_quality));
        m.set_flow_rate_x(g_mav_veh_flow_rate_x);
        m.set_flow_rate_y(g_mav_veh_flow_rate_y);
        DataLogger::publish("/mav/flow", m, g_epoch_ns);
    }

    // --- Target estimate ---
    {
        logger::TargetEstimate m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);
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
        DataLogger::publish("/target/estimate", m, g_epoch_ns);
    }

    {
        logger::Objects m;
        DataLogger::logTime(m.mutable_t(), g_epoch_ns);

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

        // Same publish style you used for TargetEstimate
        DataLogger::publish("/detection/objects", m, g_epoch_ns);
        publish("/camera/annotations", m, g_epoch_ns); // schema name must be foxglove.ImageAnnotations
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

    const std::string path = generate_unique_filename(data_file_name, ".mcap");
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
    mMCAPLogger->addChannel("/detection/objects", "logger.Objects", "protobuf");
    mMCAPLogger->addChannel("/image/annotation", "foxglove.ImageAnnotations", "protobuf");

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