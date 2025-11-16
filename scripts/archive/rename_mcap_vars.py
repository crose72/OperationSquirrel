#!/usr/bin/env python3
"""
convert_mcap_old_to_new.py

Convert "old schema" MCAP logs into "new os.logger.*" schema MCAP logs.

- Renames fields in:
    /system/state         (SystemStateMsg)
    /target/detection     (TargetDetection)
    /path/control  → /control/output (ControlOutput)

- Passes through all other topics untouched (same schema + topic).

Usage:
    python convert_mcap_old_to_new.py input.mcap
    python convert_mcap_old_to_new.py logs/*.mcap
"""

import os
import sys
import glob
from dataclasses import dataclass
from typing import Dict, Optional, Type

from mcap.reader import make_reader
from mcap.writer import Writer

# ----------------------------------------------------------------------
# 🔧 Proto imports
#   You MUST provide "old" proto modules compiled from your previous
#   schema, and "new" proto modules compiled from your current schema.
# ----------------------------------------------------------------------

# OLD schema modules (you compile these from your *old* .proto files)
import System_old_pb2    as OldSystem_pb2
import Target_old_pb2    as OldTarget_pb2
import Path_old_pb2      as OldPath_pb2

# NEW schema modules (from the .proto files you just shared)
import System_pb2        as NewSystem_pb2
import Target_pb2        as NewTarget_pb2
import Control_pb2       as NewControl_pb2
import Mavlink_pb2       as NewMavlink_pb2
import Detection_pb2     as NewDetection_pb2
import Common_pb2        as NewCommon_pb2


# ----------------------------------------------------------------------
# 🔁 Field mappings: old_field_name → new_field_name
# ----------------------------------------------------------------------

# /system/state: SystemStateMsg
SYSTEM_FIELD_MAP: Dict[str, str] = {
    # int32 system_state stayed the same
    "system_state": "system_state",
    # app_elapsed_time → app_elapsed_time_sec
    "app_elapsed_time": "app_elapsed_time_sec",
}

# /target/detection: TargetDetection
# Extracted directly from comments in Target.proto ("was: <old>")
TARGET_FIELD_MAP: Dict[str, str] = {
    "target_valid":           "is_target_detected",
    "target_detection_id":    "detection_id",
    "target_track_id":        "track_id",
    "detection_class":        "class_id",
    "target_detection_conf":  "confidence",
    "target_cntr_offset_x":   "bbox_offset_x_px",
    "target_cntr_offset_y":   "bbox_offset_y_px",
    "target_height":          "bbox_height_px",
    "target_width":           "bbox_width_px",
    "target_aspect":          "bbox_aspect_ratio",
    "target_left":            "bbox_left_px",
    "target_right":           "bbox_right_px",
    "target_top":             "bbox_top_px",
    "target_bottom":          "bbox_bottom_px",
    "target_center_x":        "bbox_center_x_px",
    "target_center_y":        "bbox_center_y_px",
    "delta_angle":            "delta_angle_deg",
    "camera_tilt_angle":      "camera_tilt_deg",
    "delta_d_x":              "delta_distance_x_m",
    "delta_d_z":              "delta_distance_z_m",
    "x_target_ekf":           "target_pos_x_est",
    "y_target_ekf":           "target_pos_y_est",
    "vx_target_ekf":          "target_vel_x_est",
    "vy_target_ekf":          "target_vel_y_est",
    "ax_target_ekf":          "target_acc_x_est",
    "ay_target_ekf":          "target_acc_y_est",
    "target_data_useful":     "is_measurement_valid",
}

# /path/control → /control/output: ControlOutput
CONTROL_FIELD_MAP: Dict[str, str] = {
    # bool target_too_close  → bool is_target_too_close
    "target_too_close": "is_target_too_close",

    # position errors
    "x_error":          "pos_error_x_m",
    "y_error":          "pos_error_y_m",

    # command velocities
    "vx_adjust":        "cmd_vel_x_mps",
    "vy_adjust":        "cmd_vel_y_mps",
    "vz_adjust":        "cmd_vel_z_mps",
    # yaw-related fields (yaw_error_rad, yaw_playback_rad, yaw_target_rad,
    # cmd_yawrate_rps) didn't exist in old logs; we'll default them to 0.
}

CONTROL_DEFAULTS: Dict[str, float] = {
    "yaw_error_rad":     0.0,
    "yaw_playback_rad":  0.0,
    "yaw_target_rad":    0.0,
    "cmd_yawrate_rps":   0.0,
}


# ----------------------------------------------------------------------
# 🧱 Migration config
# ----------------------------------------------------------------------

@dataclass
class TopicMigration:
    input_topic: str
    output_topic: str
    old_cls: Type
    new_cls: Type
    field_map: Dict[str, str]
    defaults: Optional[Dict[str, object]]
    schema_name: str       # e.g. "os.logger.SystemStateMsg"
    schema_file: str       # e.g. "System.proto"


# Define which topics to transform
MIGRATIONS: Dict[str, TopicMigration] = {
    "/system/state": TopicMigration(
        input_topic="/system/state",
        output_topic="/system/state",
        old_cls=OldSystem_pb2.SystemStateMsg,
        new_cls=NewSystem_pb2.SystemStateMsg,
        field_map=SYSTEM_FIELD_MAP,
        defaults=None,
        schema_name="os.logger.SystemStateMsg",
        schema_file="System.proto",
    ),
    "/target/detection": TopicMigration(
        input_topic="/target/detection",
        output_topic="/target/detection",
        old_cls=OldTarget_pb2.TargetDetection,
        new_cls=NewTarget_pb2.TargetDetection,
        field_map=TARGET_FIELD_MAP,
        defaults=None,
        schema_name="os.logger.TargetDetection",
        schema_file="Target.proto",
    ),
    "/path/control": TopicMigration(
        input_topic="/path/control",
        output_topic="/control/output",  # NOTE: rename topic
        old_cls=OldPath_pb2.ControlOutput,
        new_cls=NewControl_pb2.ControlOutput,
        field_map=CONTROL_FIELD_MAP,
        defaults=CONTROL_DEFAULTS,
        schema_name="os.logger.ControlOutput",
        schema_file="Control.proto",
    ),
}


# ----------------------------------------------------------------------
# 🔩 Helpers
# ----------------------------------------------------------------------

def get_proto_dir() -> str:
    proto_dir = os.environ.get("PROTO_DIR")
    if not proto_dir:
        proto_dir = os.path.dirname(os.path.abspath(__file__))
    if not os.path.isdir(proto_dir):
        raise SystemExit(f"❌ PROTO_DIR '{proto_dir}' not found.")
    return proto_dir


def copy_time_and_frame(old_msg, new_msg):
    # Copy Time t if present
    if hasattr(old_msg, "t") and hasattr(new_msg, "t"):
        try:
            new_msg.t.CopyFrom(old_msg.t)
        except Exception:
            # If types differ, fallback: copy fields if possible
            if hasattr(old_msg.t, "timestamp_ns") and hasattr(new_msg.t, "timestamp_ns"):
                new_msg.t.timestamp_ns = old_msg.t.timestamp_ns
            if hasattr(old_msg.t, "timestamp_sec") and hasattr(new_msg.t, "timestamp_sec"):
                new_msg.t.timestamp_sec = old_msg.t.timestamp_sec

    # Copy frame_id if present
    if hasattr(old_msg, "frame_id") and hasattr(new_msg, "frame_id"):
        new_msg.frame_id = getattr(old_msg, "frame_id")


def remap_fields(old_msg, new_msg, field_map: Dict[str, str], defaults: Optional[Dict[str, object]] = None):
    # Copy mapped fields
    for old_name, new_name in field_map.items():
        if hasattr(old_msg, old_name) and hasattr(new_msg, new_name):
            setattr(new_msg, new_name, getattr(old_msg, old_name))

    # Apply defaults for any extra new fields
    if defaults:
        for name, val in defaults.items():
            if hasattr(new_msg, name):
                # Only set if still default (0 / False) if you want; or always overwrite
                setattr(new_msg, name, val)


# ----------------------------------------------------------------------
# 🚀 Conversion core
# ----------------------------------------------------------------------

def convert_one_file(input_mcap: str, output_mcap: str):
    print(f"\n🧩 Converting {input_mcap} → {output_mcap}")
    proto_dir = get_proto_dir()

    with open(input_mcap, "rb") as f_in, open(output_mcap, "wb") as f_out:
        reader = make_reader(f_in)
        writer = Writer(f_out)

        # 1) Register schemas for migrated topics using new .proto text
        schema_out_ids: Dict[str, int] = {}  # migration_id → new_schema_id
        for topic, mig in MIGRATIONS.items():
            schema_path = os.path.join(proto_dir, mig.schema_file)
            with open(schema_path, "rb") as sf:
                schema_data = sf.read()
            schema_id = writer.register_schema(
                name=mig.schema_name,
                encoding="protobuf",
                data=schema_data,
            )
            schema_out_ids[topic] = schema_id

        # 2) Register schemas for pass-through topics (unchanged)
        passthrough_schema_ids = {}
        # We need channels list to know which topics use which schemas:
        channels_by_id = {ch.id: ch for ch in reader.channels}

        for schema in reader.schemas:
            # Check if this schema is used by any migrated topic
            used_topics = {ch.topic for ch in channels_by_id.values() if ch.schema_id == schema.id}
            if any(t in MIGRATIONS for t in used_topics):
                # These topics will be rewritten with new schemas instead
                continue
            new_id = writer.register_schema(
                name=schema.name,
                encoding=schema.encoding,
                data=schema.data,
            )
            passthrough_schema_ids[schema.id] = new_id

        # 3) Register channels
        passthrough_channel_ids = {}
        migration_channel_ids = {}  # topic -> new channel id

        for ch in reader.channels:
            if ch.topic in MIGRATIONS:
                mig = MIGRATIONS[ch.topic]
                if mig.input_topic not in migration_channel_ids:
                    ch_id = writer.register_channel(
                        topic=mig.output_topic,
                        schema_id=schema_out_ids[mig.input_topic],
                        message_encoding="protobuf",
                        metadata=ch.metadata,
                    )
                    migration_channel_ids[mig.input_topic] = ch_id
            else:
                new_id = writer.register_channel(
                    topic=ch.topic,
                    schema_id=passthrough_schema_ids[ch.schema_id],
                    message_encoding=ch.message_encoding,
                    metadata=ch.metadata,
                )
                passthrough_channel_ids[ch.id] = new_id

        # 4) Rewrite messages
        for schema, ch, msg in reader.iter_messages():
            topic = ch.topic

            if topic in MIGRATIONS:
                mig = MIGRATIONS[topic]

                old_obj = mig.old_cls()
                old_obj.ParseFromString(msg.data)

                new_obj = mig.new_cls()
                copy_time_and_frame(old_obj, new_obj)
                remap_fields(old_obj, new_obj, mig.field_map, defaults=mig.defaults)

                writer.write_message(
                    channel_id=migration_channel_ids[topic],
                    log_time=msg.log_time,
                    publish_time=msg.publish_time,
                    data=new_obj.SerializeToString(),
                )
            else:
                # passthrough: same data, new schema/channel ids
                writer.write_message(
                    channel_id=passthrough_channel_ids[ch.id],
                    log_time=msg.log_time,
                    publish_time=msg.publish_time,
                    data=msg.data,
                )

    print(f"✅ Done: {output_mcap}")


# ----------------------------------------------------------------------
# 🏁 CLI
# ----------------------------------------------------------------------

def main():
    if len(sys.argv) < 2:
        print("Usage: convert_mcap_old_to_new.py path/to/*.mcap")
        sys.exit(1)

    files = []
    for arg in sys.argv[1:]:
        files.extend(glob.glob(arg))

    if not files:
        print("❌ No MCAP files matched.")
        sys.exit(1)

    for path in files:
        base, ext = os.path.splitext(path)
        out = f"{base}_v2{ext}"
        convert_one_file(path, out)


if __name__ == "__main__":
    main()
