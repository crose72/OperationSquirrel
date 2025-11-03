#!/usr/bin/env python3
"""
Convert one or more MCAP files into test-harness CSV layout.

Usage:
    python mcap_to_test_csv_batch.py data_1.mcap data_2.mcap ...
or:
    python mcap_to_test_csv_batch.py /path/to/*.mcap
"""

#!/usr/bin/env python3
import csv, sys, os, glob

# --- Handle proto import path ---
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

from mcap.reader import make_reader
from collections import defaultdict
import math
import pandas as pd
proto_dir = os.environ.get("PROTO_DIR", os.path.join(os.path.dirname(__file__), "../proto"))
sys.path.append(os.path.abspath(proto_dir))

import Common_pb2, Detection_pb2, Mavlink_pb2, Path_pb2, System_pb2, Target_pb2


# --- topic→class mapping --------------------------------------------------
TOPICS = {
    "/system/state": System_pb2.SystemStateMsg,
    "/target/detection": Target_pb2.TargetDetection,
    "/target/location": Target_pb2.TargetLocation,
    "/path/control": Path_pb2.ControlOutput,
    "/mav/system": Mavlink_pb2.MavSystem,
    "/mav/kinematics": Mavlink_pb2.MavKinematics,
    "/mav/imu": Mavlink_pb2.MavImuRaw,
    "/mav/rangefinder": Mavlink_pb2.MavRangefinder,
    "/mav/flow": Mavlink_pb2.MavOpticalFlow,
    "/detection/objects": Detection_pb2.Objects,
}

# --- expected output column order ----------------------------------------
OUTPUT_COLUMNS = [
    "g_app_elapsed_time","g_system_state","g_target_too_close","g_target_valid",
    "g_target_detection_id","g_target_track_id","g_detection_class","g_target_detection_conf",
    "g_target_cntr_offset_x","g_target_cntr_offset_y",
    "g_target_cntr_offset_x_filt","g_target_cntr_offset_y_filt",
    "g_target_height","g_target_width","g_target_aspect",
    "g_target_left","g_target_right","g_target_top","g_target_bottom",
    "g_d_target_h","g_d_target_w","g_x_target","g_y_target","g_z_target","d_target",
    "g_x_error","g_y_error","g_delta_angle","g_camera_tilt_angle","g_delta_d_x",
    "g_delta_d_z","g_vx_adjust","g_vy_adjust","g_vz_adjust",
    "g_mav_veh_sys_stat_voltage_battery","g_mav_veh_sys_stat_current_battery",
    "g_mav_veh_sys_stat_battery_remaining","g_mav_veh_rel_alt",
    "g_mav_veh_gps_vx","g_mav_veh_gps_vy","g_mav_veh_gps_vz","g_mav_veh_gps_hdg",
    "g_mav_veh_roll","g_mav_veh_pitch","g_mav_veh_yaw","g_mav_veh_rollspeed",
    "g_mav_veh_pitchspeed","g_mav_veh_yawspeed","g_mav_veh_imu_ax",
    "g_mav_veh_imu_ay","g_mav_veh_imu_az","g_mav_veh_imu_xgyro",
    "g_mav_veh_imu_ygyro","g_mav_veh_imu_zgyro","g_mav_veh_rngfdr_current_distance",
    "g_mav_veh_rngfdr_signal_quality","g_mav_veh_flow_comp_m_x",
    "g_mav_veh_flow_comp_m_y","g_mav_veh_flow_x","g_mav_veh_flow_y",
    "g_mav_veh_flow_quality","g_mav_veh_flow_rate_x","g_mav_veh_flow_rate_y",
    "g_mav_veh_local_ned_x","g_mav_veh_local_ned_y","g_mav_veh_local_ned_z",
    "g_mav_veh_local_ned_vx","g_mav_veh_local_ned_vy","g_mav_veh_local_ned_vz",
    "g_mav_veh_q1_actual","g_mav_veh_q2_actual","g_mav_veh_q3_actual",
    "g_mav_veh_q4_actual","g_mav_veh_roll_rate_actual",
    "g_mav_veh_pitch_rate_actual","g_mav_veh_yaw_rate_actual",
    "g_mav_veh_repr_offset_q[0]","g_mav_veh_repr_offset_q[1]",
    "g_mav_veh_repr_offset_q[2]","g_mav_veh_repr_offset_q[3]",
    "g_mav_veh_type","g_mav_veh_autopilot_type","g_mav_veh_base_mode",
    "g_mav_veh_custom_mode","g_mav_veh_state","g_mav_veh_mavlink_version"
]

# --- flatten helper ------------------------------------------------------
def flatten(msg, prefix=""):
    result = {}
    for field, value in msg.ListFields():
        name = prefix + field.name
        # if field.label == field.LABEL_REPEATED: # deprecated - used before - worked though 10/31/2025
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


# --- process one file ----------------------------------------------------
def process_mcap(input_mcap: str):
    output_csv = os.path.splitext(input_mcap)[0] + "_test.csv"
    print(f"\n🧩 Processing {input_mcap} → {output_csv}")

    rows = []
    with open(input_mcap, "rb") as f:
        for schema, channel, message in make_reader(f).iter_messages():
            topic = channel.topic
            if topic not in TOPICS:
                continue
            msg = TOPICS[topic]()
            msg.ParseFromString(message.data)
            data = flatten(msg)
            data["timestamp_ns"] = message.log_time
            data["topic"] = topic
            rows.append(data)

    # ------------------------------------------------------------------
    # Build per-topic rows
    # ------------------------------------------------------------------
    out_rows = []
    for r in rows:
        newr = {col: 0.0 for col in OUTPUT_COLUMNS}
        t = r.get("topic", "")

        # System
        if t == "/system/state":
            newr["g_app_elapsed_time"] = r.get("app_elapsed_time", 0.0)
            newr["g_system_state"] = r.get("system_state", 0)

        # Target Detection
        elif t == "/target/detection":
            newr.update({
                "g_target_valid": r.get("target_valid", 0),
                "g_target_detection_id": r.get("target_detection_id", -1),
                "g_target_track_id": r.get("target_track_id", 0),
                "g_detection_class": r.get("detection_class", 0.0),
                "g_target_detection_conf": r.get("target_detection_conf", 0.0),
                "g_target_cntr_offset_x": r.get("target_cntr_offset_x", 0.0),
                "g_target_cntr_offset_y": r.get("target_cntr_offset_y", 0.0),
                "g_target_cntr_offset_x_filt": r.get("target_cntr_offset_x_filt", 0.0),
                "g_target_cntr_offset_y_filt": r.get("target_cntr_offset_y_filt", 0.0),
                "g_target_height": r.get("target_height", 0.0),
                "g_target_width": r.get("target_width", 0.0),
                "g_target_aspect": r.get("target_aspect", 0.0),
                "g_target_left": r.get("target_left", 0.0),
                "g_target_right": r.get("target_right", 0.0),
                "g_target_top": r.get("target_top", 0.0),
                "g_target_bottom": r.get("target_bottom", 0.0)
            })

        # Target Location
        elif t == "/target/location":
            newr.update({
                "g_d_target_h": r.get("d_target_h", 0.0),
                "g_d_target_w": r.get("d_target_w", 0.0),
                "d_target": r.get("d_target", 0.0),
                "g_x_target": r.get("x_target", 0.0),
                "g_y_target": r.get("y_target", 0.0),
                "g_z_target": r.get("z_target", 0.0),
                "g_delta_angle": r.get("delta_angle", 0.0),
                "g_camera_tilt_angle": r.get("camera_tilt_angle", 0.0),
                "g_delta_d_x": r.get("delta_d_x", 0.0),
                "g_delta_d_z": r.get("delta_d_z", 0.0),
            })

        # Control Output
        elif t == "/path/control":
            newr.update({
                "g_target_too_close": r.get("target_too_close", 0),
                "g_x_error": r.get("x_error", 0.0),
                "g_y_error": r.get("y_error", 0.0),
                "g_vx_adjust": r.get("vx_adjust", 0.0),
                "g_vy_adjust": r.get("vy_adjust", 0.0),
                "g_vz_adjust": r.get("vz_adjust", 0.0),
            })

        # MAV topics
        elif t == "/mav/system":
            newr.update({
                "g_mav_veh_sys_stat_voltage_battery": r.get("sys_stat_voltage_battery", 0),
                "g_mav_veh_sys_stat_current_battery": r.get("sys_stat_current_battery", 0),
                "g_mav_veh_sys_stat_battery_remaining": r.get("sys_stat_battery_remaining", 0),
                "g_mav_veh_rel_alt": r.get("rel_alt", 0),
                "g_mav_veh_type": r.get("veh_type", 0),
                "g_mav_veh_autopilot_type": r.get("autopilot_type", 0),
                "g_mav_veh_base_mode": r.get("base_mode", 0),
                "g_mav_veh_custom_mode": r.get("custom_mode", 0),
                "g_mav_veh_state": r.get("state", 0),
                "g_mav_veh_mavlink_version": r.get("mavlink_version", 0)
            })

        elif t == "/mav/kinematics":
            newr.update({
                # --- GPS velocities and heading ---
                "g_mav_veh_gps_vx": r.get("gps_vx", 0.0),
                "g_mav_veh_gps_vy": r.get("gps_vy", 0.0),
                "g_mav_veh_gps_vz": r.get("gps_vz", 0.0),
                "g_mav_veh_gps_hdg": r.get("gps_hdg", 0.0),

                # --- Orientation ---
                "g_mav_veh_roll": r.get("roll", 0.0),
                "g_mav_veh_pitch": r.get("pitch", 0.0),
                "g_mav_veh_yaw": r.get("yaw", 0.0),

                # --- Angular rates (deg/s or rad/s depending on source) ---
                "g_mav_veh_rollspeed": r.get("rollspeed", 0.0),
                "g_mav_veh_pitchspeed": r.get("pitchspeed", 0.0),
                "g_mav_veh_yawspeed": r.get("yawspeed", 0.0),

                # --- Local NED position ---
                "g_mav_veh_local_ned_x": r.get("local_ned_x", 0.0),
                "g_mav_veh_local_ned_y": r.get("local_ned_y", 0.0),
                "g_mav_veh_local_ned_z": r.get("local_ned_z", 0.0),

                # --- Local NED velocities ---
                "g_mav_veh_local_ned_vx": r.get("local_ned_vx", 0.0),
                "g_mav_veh_local_ned_vy": r.get("local_ned_vy", 0.0),
                "g_mav_veh_local_ned_vz": r.get("local_ned_vz", 0.0),

                # --- Quaternions ---
                "g_mav_veh_q1_actual": r.get("q1_actual", 0.0),
                "g_mav_veh_q2_actual": r.get("q2_actual", 0.0),
                "g_mav_veh_q3_actual": r.get("q3_actual", 0.0),
                "g_mav_veh_q4_actual": r.get("q4_actual", 0.0),

                # --- Rate setpoints / actual ---
                "g_mav_veh_roll_rate_actual": r.get("roll_rate_actual", 0.0),
                "g_mav_veh_pitch_rate_actual": r.get("pitch_rate_actual", 0.0),
                "g_mav_veh_yaw_rate_actual": r.get("yaw_rate_actual", 0.0),

                # --- Repr offset quaternion ---
                "g_mav_veh_repr_offset_q[0]": (
                    r.get("repr_offset_q[0]", r.get("repr_offset_q.0", 0.0))
                ),
                "g_mav_veh_repr_offset_q[1]": (
                    r.get("repr_offset_q[1]", r.get("repr_offset_q.1", 0.0))
                ),
                "g_mav_veh_repr_offset_q[2]": (
                    r.get("repr_offset_q[2]", r.get("repr_offset_q.2", 0.0))
                ),
                "g_mav_veh_repr_offset_q[3]": (
                    r.get("repr_offset_q[3]", r.get("repr_offset_q.3", 0.0))
                ),
            })

        elif t == "/mav/imu":
            newr.update({
                "g_mav_veh_imu_ax": r.get("imu_ax", 0),
                "g_mav_veh_imu_ay": r.get("imu_ay", 0),
                "g_mav_veh_imu_az": r.get("imu_az", 0),
                "g_mav_veh_imu_xgyro": r.get("imu_xgyro", 0),
                "g_mav_veh_imu_ygyro": r.get("imu_ygyro", 0),
                "g_mav_veh_imu_zgyro": r.get("imu_zgyro", 0),
            })

        elif t == "/mav/rangefinder":
            newr.update({
                "g_mav_veh_rngfdr_current_distance": r.get("current_distance", 0),
                "g_mav_veh_rngfdr_signal_quality": r.get("signal_quality", 0),
            })

        elif t == "/mav/flow":
            newr.update({
                "g_mav_veh_flow_comp_m_x": r.get("flow_comp_m_x", 0.0),
                "g_mav_veh_flow_comp_m_y": r.get("flow_comp_m_y", 0.0),
                "g_mav_veh_flow_x": r.get("flow_x", 0.0),
                "g_mav_veh_flow_y": r.get("flow_y", 0.0),
                "g_mav_veh_flow_quality": r.get("flow_quality", 0),
                "g_mav_veh_flow_rate_x": r.get("flow_rate_x", 0.0),
                "g_mav_veh_flow_rate_y": r.get("flow_rate_y", 0.0),
            })

        newr["timestamp_ns"] = r.get("timestamp_ns", 0)
        out_rows.append(newr)

    # ------------------------------------------------------------------
    # Merge by 10ms bins
    # ------------------------------------------------------------------
    BIN_NS = 1e7  # 10ms
    buckets = defaultdict(lambda: {col: 0.0 for col in OUTPUT_COLUMNS})

    for r in out_rows:
        t_ns = r.get("timestamp_ns", 0)
        key = int(round(t_ns / BIN_NS) * BIN_NS)
        row = buckets[key]
        for k, v in r.items():
            if k in OUTPUT_COLUMNS and (v != 0 or isinstance(v, str)):
                row[k] = v
        if "g_app_elapsed_time" in r:
            row["g_app_elapsed_time"] = max(row.get("g_app_elapsed_time", 0.0), r["g_app_elapsed_time"])

    merged_rows = [buckets[k] for k in sorted(buckets.keys())]

    # Convert merged_rows (list of dicts) → DataFrame
    df = pd.DataFrame(merged_rows)

    # Replace string "NaN"/"None"/"" with 0 and fill real NaN with 0
    df = df.replace(["NaN", "nan", "None", ""], 0).fillna(0)

    # Convert numeric-looking columns to actual numeric types
    df = df.apply(pd.to_numeric, errors="ignore")

    # ------------------------------------------------------------------
    # Write clean CSV
    # ------------------------------------------------------------------
    os.makedirs(os.path.dirname(output_csv) or ".", exist_ok=True)
    df.to_csv(output_csv, index=False)
    print(f"✅ Cleaned and wrote CSV → {output_csv}")

    # ------------------------------------------------------------------
    # Write CSV
    # ------------------------------------------------------------------
    os.makedirs(os.path.dirname(output_csv) or ".", exist_ok=True)
    with open(output_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=OUTPUT_COLUMNS)
        writer.writeheader()
        for r in merged_rows:
            writer.writerow(r)

    print(f"✅ Merged {len(out_rows)} → {len(merged_rows)} rows")
    print(f"✅ Wrote {output_csv}")


# --- Main entry ----------------------------------------------------------
if __name__ == "__main__":
    files = []
    for arg in sys.argv[1:]:
        files.extend(glob.glob(arg))
    if not files:
        print("❌ No MCAP files provided.")
        sys.exit(1)
    for file in files:
        process_mcap(file)
