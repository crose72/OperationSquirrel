#!/usr/bin/env python3
"""
Convert one or more MCAP files into test-harness CSV layout.

Usage:
    python mcap2CSV_config.py /path/to/*.mcap

✅ Refactored for clarity:
- Topic field mappings defined in CONFIG at the top.
- All logic is shared; no per-topic functions needed.
- Unified float defaults (0.0) for consistency.
"""

import csv, sys, os, glob, math
from collections import defaultdict
import pandas as pd
from mcap.reader import make_reader

# --------------------------------------------------------------------------
# 🔧 Proto import setup
# --------------------------------------------------------------------------
# Allow explicit override via environment variable
proto_dir = os.environ.get("PROTO_DIR")

# Otherwise, fall back to common relative paths
if not proto_dir:
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "../proto"),
        os.path.join(here, "proto"),
    ]
    for c in candidates:
        if os.path.isdir(c):
            proto_dir = c
            break

if not proto_dir or not os.path.isdir(proto_dir):
    sys.exit("❌ Could not find proto directory. Set PROTO_DIR or place one in ../proto.")

sys.path.insert(0, os.path.abspath(proto_dir))
print(f"📂 Using proto directory: {proto_dir}")

import Common_pb2, Detection_pb2, Mavlink_pb2, Control_pb2, System_pb2, Target_pb2

# --------------------------------------------------------------------------
# 📡 Topic → Proto Message Type
# --------------------------------------------------------------------------
# Each topic name (as stored in the MCAP) corresponds to a protobuf message type.
# This dictionary tells the parser which proto class to use for each topic.
TOPICS = {
    "/system/state": System_pb2.SystemStateMsg,
    "/target/detection": Target_pb2.TargetDetection,
    "/control/output": Control_pb2.ControlOutput,
    "/mav/system": Mavlink_pb2.MavSystem,
    "/mav/kinematics": Mavlink_pb2.MavKinematics,
    "/mav/imu": Mavlink_pb2.MavImuRaw,
    "/mav/rangefinder": Mavlink_pb2.MavRangefinder,
    "/mav/flow": Mavlink_pb2.MavOpticalFlow,
    "/detection/objects": Detection_pb2.Objects,
}

# --------------------------------------------------------------------------
# ⚙️ CONFIGURATION — per-topic field mappings
# --------------------------------------------------------------------------
# This maps from MCAP topic → {CSV column: protobuf field name}.
# It defines which values from each message end up as columns in the final CSV.
# You can easily add new mappings here without touching the rest of the script.
TOPIC_CONFIG = {
    "/system/state": {
        "g_app_time_s": "app_elapsed_time_sec",
        "g_system_state": "system_state",
    },

    "/target/detection": {
        "g_tgt_valid": "is_target_detected",
        "g_tgt_detect_id": "detection_id",
        "g_tgt_track_id": "track_id",
        "g_tgt_class_id": "class_id",
        "g_tgt_conf": "confidence",
        "g_tgt_cntr_offset_x_pix": "bbox_offset_x_px",
        "g_tgt_cntr_offset_y_pix": "bbox_offset_y_px",
        "g_tgt_height_pix": "bbox_height_px",
        "g_tgt_width_pix": "bbox_width_px",
        "g_tgt_aspect_ratio": "bbox_aspect_ratio",
        "g_tgt_left_px": "bbox_left_px",
        "g_tgt_right_px": "bbox_right_px",
        "g_tgt_top_px": "bbox_top_px",
        "g_tgt_bottom_px": "bbox_bottom_px",
    },

    "/target/location": {
        "g_tgt_height_meas": "d_target_h",
        "g_tgt_width_meas": "d_target_w",
        "d_target": "d_target",
        "g_tgt_pos_x_meas": "x_target",
        "g_tgt_pos_y_meas": "y_target",
        "g_tgt_pos_z_meas": "z_target",
        "g_cam0_delta_angle_deg": "delta_angle",
        "g_cam_tilt_deg": "camera_tilt_angle",
        "g_tgt_pos_x_delta": "delta_d_x",
        "g_tgt_pos_z_delta": "delta_d_z",
    },

    "/path/control": {
        "g_tgt_too_close": "target_too_close",
        "g_pos_err_x": "x_error",
        "g_pos_err_y": "y_error",
        "g_ctrl_vel_x_cmd": "vx_adjust",
        "g_ctrl_vel_y_cmd": "vy_adjust",
        "g_ctrl_vel_z_cmd": "vz_adjust",
    },

    # MAVLink system/status
    "/mav/system": {
        "g_mav_batt_voltage_mv": "sys_stat_voltage_battery",
        "g_mav_batt_current_ma": "sys_stat_current_battery",
        "g_mav_batt_remaining_pct": "sys_stat_battery_remaining",
        "g_mav_gps_alt_rel": "rel_alt",
        "g_mav_type": "veh_type",
        "g_mav_autopilot_type": "autopilot_type",
        "g_mav_mode_base": "base_mode",
        "g_mav_mode_custom": "custom_mode",
        "g_mav_state": "state",
        "g_mav_version": "mavlink_version",
    },

    # MAVLink kinematics (GPS / attitude / rates / local NED / quats)
    "/mav/kinematics": {
        # GPS velocities + heading
        "g_mav_gps_vel_x": "gps_vx",
        "g_mav_gps_vel_y": "gps_vy",
        "g_mav_gps_vel_z": "gps_vz",
        "g_mav_gps_heading_deg": "gps_hdg",

        # Orientation (Euler)
        "g_mav_veh_roll_deg": "roll",
        "g_mav_veh_pitch_deg": "pitch",
        "g_mav_veh_yaw_deg": "yaw",

        # Angular rates
        "g_mav_veh_roll_rate": "rollspeed",
        "g_mav_veh_pitch_rate": "pitchspeed",
        "g_mav_veh_yaw_rate": "yawspeed",

        # Local NED position
        "g_mav_veh_pos_ned_x": "local_ned_x",
        "g_mav_veh_pos_ned_y": "local_ned_y",
        "g_mav_veh_pos_ned_z": "local_ned_z",

        # Local NED velocities
        "g_mav_veh_vel_ned_x": "local_ned_vx",
        "g_mav_veh_vel_ned_y": "local_ned_vy",
        "g_mav_veh_vel_ned_z": "local_ned_vz",

        # Quaternions (attitude)
        "g_mav_att_actual_q1": "q1_actual",
        "g_mav_att_actual_q2": "q2_actual",
        "g_mav_att_actual_q3": "q3_actual",
        "g_mav_att_actual_q4": "q4_actual",

        # Rate setpoints / actual
        "g_mav_att_actual_roll_rate": "roll_rate_actual",
        "g_mav_att_actual_pitch_rate": "pitch_rate_actual",
        "g_mav_att_actual_yaw_rate": "yaw_rate_actual",

        # Repr offset quaternion (array fields as flattened keys)
        "g_mav_att_repr_offset_q[0]": "repr_offset_q[0]",
        "g_mav_att_repr_offset_q[1]": "repr_offset_q[1]",
        "g_mav_att_repr_offset_q[2]": "repr_offset_q[2]",
        "g_mav_att_repr_offset_q[3]": "repr_offset_q[3]",
    },

    # MAVLink IMU
    "/mav/imu": {
        "g_mav_imu_accel_x": "imu_ax",
        "g_mav_imu_accel_y": "imu_ay",
        "g_mav_imu_accel_z": "imu_az",
        "g_mav_imu_gyro_x": "imu_xgyro",
        "g_mav_imu_gyro_y": "imu_ygyro",
        "g_mav_imu_gyro_z": "imu_zgyro",
    },

    # MAVLink rangefinder
    "/mav/rangefinder": {
        "g_mav_rngfndr_dist_m": "current_distance",
        "g_mav_rngfndr_quality": "signal_quality",
    },

    # MAVLink optical flow
    "/mav/flow": {
        "g_mav_flow_vel_x": "flow_comp_m_x",
        "g_mav_flow_vel_y": "flow_comp_m_y",
        "g_mav_flow_px_x": "flow_x",
        "g_mav_flow_px_y": "flow_y",
        "g_mav_flow_quality": "flow_quality",
        "g_mav_flow_rate_x": "flow_rate_x",
        "g_mav_flow_rate_y": "flow_rate_y",
    },
}

# --------------------------------------------------------------------------
# 🧱 Helpers
# --------------------------------------------------------------------------
def flatten(msg, prefix=""):
    """Recursively flatten protobuf messages into a dict."""
    result = {}
    for field, value in msg.ListFields():
        name = prefix + field.name
        if field.is_repeated:
            if field.type == field.TYPE_MESSAGE:
                for i, submsg in enumerate(value):
                    result.update(flatten(submsg, f"{name}[{i}]."))
            else:
                result[name] = ",".join(str(v) for v in value)
        elif field.type == field.TYPE_MESSAGE:
            result.update(flatten(value, name + "."))
        else:
            result[name] = value
    return result

# --------------------------------------------------------------------------
# 🧩 Process one MCAP file
# --------------------------------------------------------------------------
def process_mcap(input_mcap: str):
    """Read one .mcap file, extract selected data, and export it to CSV."""
    output_csv = os.path.splitext(input_mcap)[0] + "_test.csv"
    print(f"\n🧩 Processing {input_mcap} → {output_csv}")

    # Step 1: Parse every message from the MCAP log.
    # Each MCAP file contains a series of "channels" (topics),
    # each with binary-encoded protobuf messages.
    raw_rows = []
    with open(input_mcap, "rb") as f:
        for schema, channel, message in make_reader(f).iter_messages():
            topic = channel.topic
            if topic not in TOPICS:
                # Skip topics we don't recognize.
                continue
            # Create the appropriate protobuf object for this topic.
            msg = TOPICS[topic]()
            # Deserialize raw bytes into that protobuf message.
            msg.ParseFromString(message.data)
            # Flatten into a dict of {field_name: value}
            data = flatten(msg)
            # Record the timestamp (ns since epoch) and topic.
            data["timestamp_ns"] = message.log_time
            data["topic"] = topic
            raw_rows.append(data)

    # Step 2: Convert each raw dict to a standardized CSV-ready format.
    # We use the TOPIC_CONFIG mapping above to decide which fields to include.
    processed = []
    for r in raw_rows:
        topic = r.get("topic", "")
        newr = {}
        for csv_field, proto_field in TOPIC_CONFIG.get(topic, {}).items():
            newr[csv_field] = r.get(proto_field, 0.0)
        newr["timestamp_ns"] = r.get("timestamp_ns", 0.0)
        processed.append(newr)

    # Step 3: Merge messages into 10ms time bins.
    # This smooths out timing differences between asynchronous topics.
    BIN_NS = 1e7  # 10 ms in nanoseconds
    buckets = defaultdict(dict)
    for row in processed:
        # Round timestamp to the nearest 10ms bin.
        key = int(round(row["timestamp_ns"] / BIN_NS) * BIN_NS)
        merged = buckets[key]
        # For each field, only overwrite if it has a nonzero or non-empty value.
        for k, v in row.items():
            # Keep any value that's not None, NaN, or an empty string
            if v is not None and not (isinstance(v, float) and math.isnan(v)) and v != "":
                merged[k] = v


    # Step 4: Convert merged data into a DataFrame (table).
    # Missing values → 0, NaN → 0, all numbers parsed correctly.
    df = pd.DataFrame([buckets[k] for k in sorted(buckets.keys())])
    df = df.replace(["NaN", "nan", "None", ""], 0).fillna(0)
    df = df.apply(pd.to_numeric, errors="ignore")

    # Step 5: Write the final, clean CSV file.
    df.to_csv(output_csv, index=False)
    print(f"✅ Wrote {len(df)} merged rows → {output_csv}")

# --------------------------------------------------------------------------
# 🚀 Main
# --------------------------------------------------------------------------
if __name__ == "__main__":
    files = []
    for arg in sys.argv[1:]:
        files.extend(glob.glob(arg))
    if not files:
        sys.exit("❌ No MCAP files provided.")
    for f in files:
        process_mcap(f)
