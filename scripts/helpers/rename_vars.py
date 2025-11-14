#!/usr/bin/env python3
"""
bulk_rename_signals.py
Renames OperationSquirrel/SquirrelDefender globals to a consistent convention.

Usage:
  # Dry run (default): shows planned replacements
  python3 bulk_rename_signals.py /path/to/OperationSquirrel

  # Actually apply changes:
  python3 bulk_rename_signals.py /path/to/OperationSquirrel --apply

WARNING:
  Commit your repo BEFORE running with --apply.
  git add -A && git commit -m "pre-rename checkpoint"

This script will rewrite thousands of identifiers. Be safe.
"""

import argparse
import pathlib
import re
import sys

# File extensions to process
EXTS = {
    ".csv"
}

# --------------------------------------------------------------------------
# COMPLETE RENAME MAP (ALL MAV + detection + tracking + control + camera)
# --------------------------------------------------------------------------
RENAME_MAP = {

    # -------------------- DETECTION --------------------
    "g_net":                        "g_det_nv_net",
    "g_detections":                 "g_det_nv_list",
    "g_detection_count":            "g_det_nv_count",

    "g_yolo_detections":            "g_det_yolo_list",
    "g_yolo_detection_count":       "g_det_yolo_count",

    # -------------------- GLOBAL OBJECTS / APP -------------------
    "g_stop_program":               "g_app_stop",
    "g_use_video_playback":         "g_app_use_video_playback",
    "g_input_video_path":           "g_app_video_input_path",

    # -------------------- LOCALIZE TARGET ------------------------
    "g_d_target_h":                 "g_tgt_height_meas",
    "g_d_target_w":                 "g_tgt_width_meas",
    "g_x_target":                   "g_tgt_pos_x_meas",
    "g_y_target":                   "g_tgt_pos_y_meas",
    "g_z_target":                   "g_tgt_pos_z_meas",
    "g_d_target":                   "g_tgt_dist_meas",

    # Mismatch-fixed → use (A) = real header names
    "g_delta_angle":                "g_cam0_delta_angle_deg",
    "g_camera_tilt_angle":          "g_cam_tilt_deg",
    "g_delta_d_x":                  "g_tgt_pos_x_delta",
    "g_delta_d_z":                  "g_tgt_pos_z_delta",
    "g_camera_comp_angle":          "g_cam0_comp_angle_deg",

    "g_x_target_ekf":               "g_tgt_pos_x_est",
    "g_y_target_ekf":               "g_tgt_pos_y_est",
    "g_vx_target_ekf":              "g_tgt_vel_x_est",
    "g_vy_target_ekf":              "g_tgt_vel_y_est",
    "g_ax_target_ekf":              "g_tgt_acc_x_est",
    "g_ay_target_ekf":              "g_tgt_acc_y_est",

    "g_target_data_useful":         "g_tgt_meas_valid",

    # Mismatch-fixed
    "g_fov_height":                 "g_cam0_fov_height",
    "g_meter_per_pix":              "g_cam0_m_per_pix",

    "g_target_cntr_offset_x_m":     "g_tgt_cntr_offset_x_m",
    "g_target_cntr_offset_x_mov_avg": "g_tgt_cntr_offset_x_filt",
    "g_target_cntr_offset_y_mov_avg": "g_tgt_cntr_offset_y_filt",

    # Mismatch-fixed
    "g_line_of_sight":              "g_cam0_los_m",

    "g_target_is_lost":             "g_tgt_lost",
    "g_target_lost_dbc_sec":        "g_tgt_lost_dbc_sec",

    # -------------------- PATH PLANNER / CONTROL -----------------
    "g_target_too_close":           "g_tgt_too_close",

    # These were kept because they match your new headers:
    "g_x_error":                    "g_pos_err_x",
    "g_y_error":                    "g_pos_err_y",

    "g_vx_adjust":                  "g_ctrl_vel_x_cmd",
    "g_vy_adjust":                  "g_ctrl_vel_y_cmd",
    "g_vz_adjust":                  "g_ctrl_vel_z_cmd",
    "g_yaw_target":                 "g_ctrl_yaw_tgt",
    "g_yaw_adjust":                 "g_ctrl_yaw_cmd",
    "g_mav_veh_yaw_adjusted_for_playback": "g_veh_yaw_playback_adj",
    "g_yaw_target_error":           "g_yaw_err",
    "g_veh_vx_est":                 "g_veh_vel_x_est",
    "g_veh_vy_est":                 "g_veh_vel_y_est",
    "g_x_error_dot":                "g_pos_err_x_dot",

    # -------------------- SCHEDULER / SYSTEM / TIME --------------
    "g_controller_initialiazed":    "g_system_init",
    "g_manual_override_land":       "g_ctrl_land_override",

    "g_app_elapsed_time":           "g_app_time_s",
    "g_app_elapsed_time_ns":        "g_app_time_ns",
    "g_dt":                         "g_app_dt",
    "g_first_loop_after_start":     "g_app_first_loop",
    "g_epoch_ns":                   "g_app_epoch_ns",

    # -------------------- TRACK TARGET ---------------------------
    "g_target_valid":               "g_tgt_valid",
    "g_target_detection_id":        "g_tgt_detect_id",
    "g_target_track_id":            "g_tgt_track_id",

    "g_target_cntr_offset_x":       "g_tgt_cntr_offset_x_pix",
    "g_target_cntr_offset_y":       "g_tgt_cntr_offset_y_pix",

    "g_target_height":              "g_tgt_height_pix",
    "g_target_width":               "g_tgt_width_pix",
    "g_target_aspect":              "g_tgt_aspect_ratio",

    "g_target_left":                "g_tgt_left_px",
    "g_target_right":               "g_tgt_right_px",
    "g_target_top":                 "g_tgt_top_px",
    "g_target_bottom":              "g_tgt_bottom_px",
    "g_target_center_x":            "g_tgt_center_x_pix",
    "g_target_center_y":            "g_tgt_center_y_pix",

    "g_detection_class":            "g_tgt_class_id",
    "g_target_detection_conf":      "g_tgt_conf",

    "g_target_cntr_offset_x_filt":  "g_tgt_cntr_offset_x_filt",
    "g_target_cntr_offset_y_filt":  "g_tgt_cntr_offset_y_filt",

    # -------------------- VIDEO / CAMERA -------------------------
    "g_cam0_valid_image_rcvd":      "g_cam0_img_valid",
    "g_cam0_image":                 "g_cam0_img_cpu",
    "g_cam0_image_gpu":             "g_cam0_img_gpu",

    "g_cam0_video_width":           "g_cam0_img_width_px",
    "g_cam0_video_height":          "g_cam0_img_height_px",

    "g_camera_fov":                 "g_cam0_fov_deg",

    "g_cam0_video_width_center":    "g_cam0_img_width_cx",
    "g_cam0_video_height_center":   "g_cam0_img_height_cy",

    "g_cam0_fov_rad":               "g_cam0_fov_rad",
    "g_cam0_fov_rad_half":          "g_cam0_fov_rad_half",
    "g_cam0_tilt_down_angle":       "g_cam0_tilt_deg",
    "g_cam0_tilt_down_angle_rad":   "g_cam0_tilt_rad",

    "g_end_of_video":               "g_video_end",
    "g_cam0_frame_id":              "g_cam0_frame_id",

    # ----------------------------------------------------------------------
    #                           MAVLINK (all signals)
    # ----------------------------------------------------------------------

    # --- Command Feedback ---
    "g_mav_veh_command_id":                "g_mav_cmd_id",
    "g_mav_veh_command_result":            "g_mav_cmd_result",
    "g_mav_veh_command_progress":          "g_mav_cmd_progress",
    "g_mav_veh_command_result_param2":     "g_mav_cmd_result_param2",
    "g_mav_veh_command_target_system":     "g_mav_cmd_tgt_sys",
    "g_mav_veh_command_target_component":  "g_mav_cmd_tgt_comp",

    # --- System Status ---
    "g_mav_veh_sys_stat_onbrd_cntrl_snsrs_present": "g_mav_sys_sensors_present",
    "g_mav_veh_sys_stat_onbrd_cntrl_snsrs_enabled": "g_mav_sys_sensors_enabled",
    "g_mav_veh_sys_stat_onbrd_cntrl_snsrs_health":  "g_mav_sys_sensors_health",
    "g_mav_veh_sys_stat_load":                       "g_mav_sys_load",
    "g_mav_veh_sys_stat_voltage_battery":            "g_mav_batt_voltage_mv",
    "g_mav_veh_sys_stat_current_battery":            "g_mav_batt_current_ma",
    "g_mav_veh_sys_stat_drop_rate_comm":             "g_mav_comm_drop_rate",
    "g_mav_veh_sys_stat_errors_comm":                "g_mav_comm_errors",
    "g_mav_veh_sys_stat_errors_count1":              "g_mav_err_count1",
    "g_mav_veh_sys_stat_errors_count2":              "g_mav_err_count2",
    "g_mav_veh_sys_stat_errors_count3":              "g_mav_err_count3",
    "g_mav_veh_sys_stat_errors_count4":              "g_mav_err_count4",
    "g_mav_veh_sys_stat_battery_remaining":          "g_mav_batt_remaining_pct",
    "g_mav_veh_sys_stat_onbrd_cntrl_snsrs_prsnt_extnd": "g_mav_sys_sensors_present_ext",
    "g_mav_veh_sys_stat_onbrd_cntrl_snsrs_enbld_extnd": "g_mav_sys_sensors_enabled_ext",
    "g_mav_veh_sys_stat_onbrd_cntrl_snsrs_health_extnd":"g_mav_sys_sensors_health_ext",

    # --- Vehicle Meta ---
    "g_mav_veh_custom_mode":     "g_mav_mode_custom",
    "g_mav_veh_type":            "g_mav_type",
    "g_mav_veh_autopilot_type":  "g_mav_autopilot_type",
    "g_mav_veh_base_mode":       "g_mav_mode_base",
    "g_mav_veh_state":           "g_mav_state",
    "g_mav_veh_mavlink_version": "g_mav_version",

    # --- GPS ---
    "g_mav_veh_lat":      "g_mav_gps_lat",
    "g_mav_veh_lon":      "g_mav_gps_lon",
    "g_mav_veh_alt":      "g_mav_gps_alt_msl",
    "g_mav_veh_rel_alt":  "g_mav_gps_alt_rel",
    "g_mav_veh_gps_vx":   "g_mav_gps_vel_x",
    "g_mav_veh_gps_vy":   "g_mav_gps_vel_y",
    "g_mav_veh_gps_vz":   "g_mav_gps_vel_z",
    "g_mav_veh_gps_hdg":  "g_mav_gps_heading_deg",

    # --- Euler Angles ---
    "g_mav_veh_roll":   "g_mav_veh_roll_deg",
    "g_mav_veh_pitch":  "g_mav_veh_pitch_deg",
    "g_mav_veh_yaw":    "g_mav_veh_yaw_deg",

    # --- Angular Rates ---
    "g_mav_veh_rollspeed":   "g_mav_veh_roll_rate",
    "g_mav_veh_pitchspeed":  "g_mav_veh_pitch_rate",
    "g_mav_veh_yawspeed":    "g_mav_veh_yaw_rate",

    # --- IMU Raw ---
    "g_mav_veh_imu_ax":     "g_mav_imu_accel_x",
    "g_mav_veh_imu_ay":     "g_mav_imu_accel_y",
    "g_mav_veh_imu_az":     "g_mav_imu_accel_z",
    "g_mav_veh_imu_xgyro":  "g_mav_imu_gyro_x",
    "g_mav_veh_imu_ygyro":  "g_mav_imu_gyro_y",
    "g_mav_veh_imu_zgyro":  "g_mav_imu_gyro_z",
    "g_mav_veh_imu_xmag":   "g_mav_imu_mag_x",
    "g_mav_veh_imu_ymag":   "g_mav_imu_mag_y",
    "g_mav_veh_imu_zmag":   "g_mav_imu_mag_z",

    # --- Attitude Target ---
    "g_mav_veh_q1_target":           "g_mav_att_target_q1",
    "g_mav_veh_q2_target":           "g_mav_att_target_q2",
    "g_mav_veh_q3_target":           "g_mav_att_target_q3",
    "g_mav_veh_q4_target":           "g_mav_att_target_q4",
    "g_mav_veh_roll_rate_target":    "g_mav_att_target_roll_rate",
    "g_mav_veh_pitch_rate_target":   "g_mav_att_target_pitch_rate",
    "g_mav_veh_yaw_rate_target":     "g_mav_att_target_yaw_rate",
    "g_mav_veh_thrust_target":       "g_mav_att_target_thrust",

    # --- Attitude Actual ---
    "g_mav_veh_q1_actual":     "g_mav_att_actual_q1",
    "g_mav_veh_q2_actual":     "g_mav_att_actual_q2",
    "g_mav_veh_q3_actual":     "g_mav_att_actual_q3",
    "g_mav_veh_q4_actual":     "g_mav_att_actual_q4",
    "g_mav_veh_roll_rate_actual":  "g_mav_att_actual_roll_rate",
    "g_mav_veh_pitch_rate_actual": "g_mav_att_actual_pitch_rate",
    "g_mav_veh_yaw_rate_actual":   "g_mav_att_actual_yaw_rate",
    "g_mav_veh_repr_offset_q":     "g_mav_att_repr_offset_q",

    # --- Rangefinder ---
    "g_mav_veh_rngfdr_min_distance":       "g_mav_rngfndr_min_cm",
    "g_mav_veh_rngfdr_max_distance":       "g_mav_rngfndr_max_cm",
    "g_mav_veh_rngfdr_current_distance":   "g_mav_rngfndr_dist_m",
    "g_mav_veh_rngfdr_type":               "g_mav_rngfndr_type",
    "g_mav_veh_rngfdr_id":                 "g_mav_rngfndr_id",
    "g_mav_veh_rngfdr_orientation":        "g_mav_rngfndr_orient",
    "g_mav_veh_rngfdr_covariance":         "g_mav_rngfndr_cov",
    "g_mav_veh_rngfdr_horizontal_fov":     "g_mav_rngfndr_fov_horiz_deg",
    "g_mav_veh_rngfdr_vertical_fov":       "g_mav_rngfndr_fov_vert_deg",
    "g_mav_veh_rngfdr_quaternion":         "g_mav_rngfndr_quat",
    "g_mav_veh_rngfdr_signal_quality":     "g_mav_rngfndr_quality",

    # --- Optical Flow ---
    "g_mav_veh_flow_comp_m_x":   "g_mav_flow_vel_x",
    "g_mav_veh_flow_comp_m_y":   "g_mav_flow_vel_y",
    "g_mav_veh_ground_distance": "g_mav_flow_ground_dist_m",
    "g_mav_veh_flow_x":          "g_mav_flow_px_x",
    "g_mav_veh_flow_y":          "g_mav_flow_px_y",
    "g_mav_veh_sensor_id":       "g_mav_flow_sensor_id",
    "g_mav_veh_flow_quality":    "g_mav_flow_quality",
    "g_mav_veh_flow_rate_x":     "g_mav_flow_rate_x",
    "g_mav_veh_flow_rate_y":     "g_mav_flow_rate_y",

    # --- Local Position NED ---
    "g_mav_veh_local_ned_x":    "g_mav_veh_pos_ned_x",
    "g_mav_veh_local_ned_y":    "g_mav_veh_pos_ned_y",
    "g_mav_veh_local_ned_z":    "g_mav_veh_pos_ned_z",
    "g_mav_veh_local_ned_vx":   "g_mav_veh_vel_ned_x",
    "g_mav_veh_local_ned_vy":   "g_mav_veh_vel_ned_y",
    "g_mav_veh_local_ned_vz":   "g_mav_veh_vel_ned_z",
}

# ------------------------------------------------------------------------------

#!/usr/bin/env python3
"""
SAFE bulk renamer for OperationSquirrel/SquirrelDefender.

✔ Only scans ALLOWED folders
✔ Explicitly excludes mavlink/ autogenerated code
✔ Will not touch anything else in the repo
✔ Dry-run by default; use --apply to modify files
"""

import argparse
import pathlib
import re
import sys

# --------------------------------------------------------------------------
# ALLOWLIST: only these paths will be scanned
# --------------------------------------------------------------------------
ALLOW_DIRS = [
    "scripts/mcap_data",
]

# BLOCKLIST: explicitly exclude mavlink-related code
EXCLUDE_DIRS = [
    "SquirrelDefender/",
]

# --------------------------------------------------------------------------
# Helper: should this file be processed?
# --------------------------------------------------------------------------
def file_in_allowed_dir(path: pathlib.Path, repo_root: pathlib.Path) -> bool:
    """Check if file is inside an allowlisted dir and not in excluded dirs."""
    rel = path.relative_to(repo_root).as_posix()

    # must start with at least one allowed directory
    if not any(rel.startswith(d) for d in ALLOW_DIRS):
        return False

    # explicitly exclude mavlink folders
    if any(rel.startswith(b) for b in EXCLUDE_DIRS):
        return False

    return True

def should_process(path: pathlib.Path, repo_root: pathlib.Path) -> bool:
    if path.is_dir():
        return False
    if not file_in_allowed_dir(path, repo_root):
        return False
    if path.name == "CMakeLists.txt":
        return True
    if path.suffix in EXTS:
        return True
    return False

# --------------------------------------------------------------------------
# MAIN
# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root", help="Path to repo root (OperationSquirrel)")
    ap.add_argument("--apply", action="store_true",
                    help="Actually write changes (default: dry-run)")
    args = ap.parse_args()

    root = pathlib.Path(args.root).resolve()
    if not root.exists():
        print(f"ERROR: Root path not found: {root}")
        sys.exit(1)

    # longest keys first to avoid substring collisions
    keys = sorted(RENAME_MAP.keys(), key=len, reverse=True)
    patterns = [(re.compile(rf"\b{re.escape(k)}\b"), RENAME_MAP[k])
                for k in keys]

    total_scanned = 0
    total_changed = 0
    total_hits = 0

    for path in root.rglob("*"):
        if not should_process(path, root):
            continue

        total_scanned += 1

        try:
            text = path.read_text(encoding="utf-8")
        except Exception:
            continue

        file_hits = 0
        new_text = text

        for rx, repl in patterns:
            matches = list(rx.finditer(new_text))
            file_hits += len(matches)
            new_text = rx.sub(repl, new_text)

        if file_hits > 0:
            total_changed += 1
            total_hits += file_hits
            print(f"→ {path}  ({file_hits} matches)")

            if args.apply:
                try:
                    path.write_text(new_text, encoding="utf-8")
                except Exception as e:
                    print(f"  !! WRITE FAILED: {e}")

    print("\n=== SUMMARY ===")
    print(f"Files scanned: {total_scanned}")
    print(f"Files changed: {total_changed}")
    print(f"Total replacements: {total_hits}")
    print("Mode:", "APPLIED" if args.apply else "DRY-RUN")

    if not args.apply:
        print("\nNo files modified. Run with --apply to perform the rename.")


if __name__ == "__main__":
    main()
