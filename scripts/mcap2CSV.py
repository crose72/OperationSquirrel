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

import Common_pb2, Detection_pb2, Mavlink_pb2, Path_pb2, System_pb2, Target_pb2

# --------------------------------------------------------------------------
# 📡 Topic → Proto Message Type
# --------------------------------------------------------------------------
# Each topic name (as stored in the MCAP) corresponds to a protobuf message type.
# This dictionary tells the parser which proto class to use for each topic.
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

# --------------------------------------------------------------------------
# ⚙️ CONFIGURATION — per-topic field mappings
# --------------------------------------------------------------------------
# This maps from MCAP topic → {CSV column: protobuf field name}.
# It defines which values from each message end up as columns in the final CSV.
# You can easily add new mappings here without touching the rest of the script.
TOPIC_CONFIG = {
    "/system/state": {
        "g_app_elapsed_time": "app_elapsed_time",
        "g_system_state": "system_state",
    },
    "/target/detection": {
        "g_target_valid": "target_valid",
        "g_target_detection_id": "target_detection_id",
        "g_target_track_id": "target_track_id",
        "g_detection_class": "detection_class",
        "g_target_detection_conf": "target_detection_conf",
        "g_target_cntr_offset_x": "target_cntr_offset_x",
        "g_target_cntr_offset_y": "target_cntr_offset_y",
        "g_target_cntr_offset_x_filt": "target_cntr_offset_x_filt",
        "g_target_cntr_offset_y_filt": "target_cntr_offset_y_filt",
        "g_target_height": "target_height",
        "g_target_width": "target_width",
        "g_target_aspect": "target_aspect",
        "g_target_left": "target_left",
        "g_target_right": "target_right",
        "g_target_top": "target_top",
        "g_target_bottom": "target_bottom",
    },
    "/target/location": {
        "g_d_target_h": "d_target_h",
        "g_d_target_w": "d_target_w",
        "d_target": "d_target",
        "g_x_target": "x_target",
        "g_y_target": "y_target",
        "g_z_target": "z_target",
        "g_delta_angle": "delta_angle",
        "g_camera_tilt_angle": "camera_tilt_angle",
        "g_delta_d_x": "delta_d_x",
        "g_delta_d_z": "delta_d_z",
    },
    "/path/control": {
        "g_target_too_close": "target_too_close",
        "g_x_error": "x_error",
        "g_y_error": "y_error",
        "g_vx_adjust": "vx_adjust",
        "g_vy_adjust": "vy_adjust",
        "g_vz_adjust": "vz_adjust",
    },
    "/mav/rangefinder": {
        "g_mav_veh_rngfdr_current_distance": "current_distance",
        "g_mav_veh_rngfdr_signal_quality": "signal_quality",
    },
    "/mav/flow": {
        "g_mav_veh_flow_comp_m_x": "flow_comp_m_x",
        "g_mav_veh_flow_comp_m_y": "flow_comp_m_y",
        "g_mav_veh_flow_x": "flow_x",
        "g_mav_veh_flow_y": "flow_y",
        "g_mav_veh_flow_quality": "flow_quality",
        "g_mav_veh_flow_rate_x": "flow_rate_x",
        "g_mav_veh_flow_rate_y": "flow_rate_y",
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
    output_csv = os.path.splitext(input_mcap)[0] + "_test.csv"
    print(f"\n🧩 Processing {input_mcap} → {output_csv}")

    # --- Parse all messages ---
    raw_rows = []
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
            raw_rows.append(data)

    # --- Convert to unified format ---
    processed = []
    for r in raw_rows:
        topic = r.get("topic", "")
        newr = {}
        for out_field, src_field in TOPIC_CONFIG.get(topic, {}).items():
            newr[out_field] = r.get(src_field, 0.0)
        newr["timestamp_ns"] = r.get("timestamp_ns", 0.0)
        processed.append(newr)

    # --- Merge by 10ms bins ---
    BIN_NS = 1e7  # 10ms
    buckets = defaultdict(dict)
    for row in processed:
        key = int(round(row["timestamp_ns"] / BIN_NS) * BIN_NS)
        merged = buckets[key]
        for k, v in row.items():
            if v != 0.0 or isinstance(v, str):
                merged[k] = v

    df = pd.DataFrame([buckets[k] for k in sorted(buckets.keys())])
    df = df.replace(["NaN", "nan", "None", ""], 0).fillna(0)
    df = df.apply(pd.to_numeric, errors="ignore")
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
