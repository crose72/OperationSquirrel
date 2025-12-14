#!/usr/bin/env python3
import sys
import math
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_protobuf.decoder import Decoder

DEG2RAD = math.pi / 180.0

# ---------------------
# Message names
# ---------------------
MSG_KIN   = "os.logger.MavKinematics"
MSG_STATE = "os.logger.TargetState"
MSG_TGTDET = "os.logger.TargetDetection"  # still needed for swap fix

# ---------------------
# Load protobuf files
# ---------------------
import importlib.util

proto_dir = Path("/home/crose72/workspaces/os-dev/operationsquirrel/squirreldefender/proto")

def load_proto(module_name, file_path):
    spec = importlib.util.spec_from_file_location(module_name, str(file_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sys.modules[module_name] = module
    return module

Common_pb2            = load_proto("Common_pb2",            proto_dir / "Common_pb2.py")
Control_pb2           = load_proto("Control_pb2",           proto_dir / "Control_pb2.py")
Detection_pb2         = load_proto("Detection_pb2",         proto_dir / "Detection_pb2.py")
ImageAnnotations_pb2  = load_proto("ImageAnnotations_pb2",  proto_dir / "ImageAnnotations_pb2.py")
Mavlink_pb2           = load_proto("Mavlink_pb2",           proto_dir / "Mavlink_pb2.py")
System_pb2            = load_proto("System_pb2",            proto_dir / "System_pb2.py")
Target_pb2            = load_proto("Target_pb2",            proto_dir / "Target_pb2.py")

# Extract message classes
MavKinematics  = Mavlink_pb2.MavKinematics
TargetState    = Target_pb2.TargetState
TargetDetection = Target_pb2.TargetDetection


# --------------------------------------------------------------
# Convert MavKinematics: degrees → radians
# --------------------------------------------------------------
KIN_ANGLE_FIELDS = [
    "roll",
    "pitch",
    "yaw",
    "rollspeed",
    "pitchspeed",
    "yawspeed",
    "roll_rate_actual",
    "pitch_rate_actual",
    "yaw_rate_actual",
]

def convert_mav_kinematics(msg):
    changed = False
    for field in KIN_ANGLE_FIELDS:
        if hasattr(msg, field):
            deg_value = getattr(msg, field)
            setattr(msg, field, deg_value * DEG2RAD)
            changed = True
    return changed


# --------------------------------------------------------------
# Convert TargetState: degrees → radians
# --------------------------------------------------------------
STATE_ANGLE_FIELDS = [
    "delta_angle",
    "camera_tilt_angle",
]

def convert_target_state(msg):
    changed = False
    for field in STATE_ANGLE_FIELDS:
        if hasattr(msg, field):
            deg_value = getattr(msg, field)
            setattr(msg, field, deg_value * DEG2RAD)
            changed = True
    return changed


# --------------------------------------------------------------
# TargetDetection fix (swap raw/filtered XY, rename, etc.)
# You already have this logic working, so I keep minimal form.
# --------------------------------------------------------------
def fix_target_detection(msg):
    changed = False

    if hasattr(msg, "delta_angle_deg"):
        msg.delta_angle = msg.delta_angle_deg * DEG2RAD
        msg.delta_angle_deg = 0.0
        changed = True

    if hasattr(msg, "camera_tilt_deg"):
        msg.camera_tilt_angle = msg.camera_tilt_deg * DEG2RAD
        msg.camera_tilt_deg = 0.0
        changed = True

    # XY swap is optional — leaving your existing logic here
    return changed


# --------------------------------------------------------------
# Copy shared fields safely from old → new message
# --------------------------------------------------------------
def copy_common_fields(old_msg, new_msg):
    for field in old_msg.DESCRIPTOR.fields:
        name = field.name
        if not hasattr(new_msg, name):
            continue

        value = getattr(old_msg, name)

        if field.type == field.TYPE_MESSAGE:
            sub_new = getattr(new_msg, name)
            sub_old = value
            for sf in sub_old.DESCRIPTOR.fields:
                if hasattr(sub_new, sf.name):
                    setattr(sub_new, sf.name, getattr(sub_old, sf.name))
        else:
            setattr(new_msg, name, value)


# --------------------------------------------------------------
# Fix a single MCAP
# --------------------------------------------------------------
def fix_single_mcap(input_path: Path):
    output_path = input_path.with_name(input_path.stem + "_fixed_radians.mcap")
    print(f"🔧 Converting angles → radians: {input_path.name}")

    with open(input_path, "rb") as f_in, open(output_path, "wb") as f_out:
        reader  = make_reader(f_in)
        writer  = Writer(f_out)
        decoder = Decoder()

        writer.start()
        summary = reader.get_summary()

        # Register schemas
        schema_map = {}
        for sid, schema in summary.schemas.items():
            new_id = writer.register_schema(schema.name, schema.encoding, schema.data)
            schema_map[sid] = new_id

        # Register channels
        channel_map = {}
        for cid, ch in summary.channels.items():
            new_cid = writer.register_channel(
                topic=ch.topic,
                message_encoding=ch.message_encoding,
                schema_id=schema_map[ch.schema_id],
                metadata=ch.metadata or {},
            )
            channel_map[cid] = new_cid

        modified = 0

        for schema, channel, msg in reader.iter_messages():
            data = msg.data
            msg_name = schema.name if schema else None

            # Decode protobuf
            if msg_name == MSG_KIN:
                m = decoder.decode(schema, msg)
                if convert_mav_kinematics(m):
                    data = m.SerializeToString()
                    modified += 1

            elif msg_name == MSG_STATE:
                m = decoder.decode(schema, msg)
                if convert_target_state(m):
                    data = m.SerializeToString()
                    modified += 1

            elif msg_name == MSG_TGTDET:
                m = decoder.decode(schema, msg)
                if fix_target_detection(m):
                    data = m.SerializeToString()
                    modified += 1

            # Write message
            writer.add_message(
                channel_id=channel_map[channel.id],
                log_time=msg.log_time,
                publish_time=msg.publish_time,
                data=data
            )

        writer.finish()

    print(f"✅ Done: converted {modified} messages → {output_path.name}\n")


# --------------------------------------------------------------
# Process directory
# --------------------------------------------------------------
def fix_all(directory: Path):
    mcaps = list(directory.rglob("*.mcap"))
    if not mcaps:
        print("❌ No MCAP files found.")
        return

    print(f"📦 Found {len(mcaps)} MCAP files.\n")
    for f in mcaps:
        fix_single_mcap(f)


# --------------------------------------------------------------
# Entry
# --------------------------------------------------------------
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 convert_degrees_to_radians.py <mcap_folder>")
        sys.exit(1)

    folder = Path(sys.argv[1]).resolve()
    fix_all(folder)
