#!/usr/bin/env python3
import sys
import math
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_protobuf.decoder import Decoder

TARGET_DET_MSG = "os.logger.TargetDetection"
MAV_KIN_MSG    = "os.logger.MavKinematics"

DEG_PER_RAD = 180.0 / math.pi

# ---------------------------------------
#  Forced import of protobuf modules
# ---------------------------------------
import importlib.util
import sys
from pathlib import Path

proto_dir = Path("/home/crose72/workspaces/os-dev/operationsquirrel/squirreldefender/proto")

def load_proto(module_name: str, file_path: Path):
    """Load a protobuf Python file manually and register it in sys.modules."""
    spec = importlib.util.spec_from_file_location(module_name, str(file_path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    # 🔑 Make sure other modules can import it normally
    sys.modules[module_name] = module

    return module


# ---------------------------------------
#  Load all protobuf dependencies FIRST
# ---------------------------------------
Common_pb2            = load_proto("Common_pb2",            proto_dir / "Common_pb2.py")
Control_pb2           = load_proto("Control_pb2",           proto_dir / "Control_pb2.py")
Detection_pb2         = load_proto("Detection_pb2",         proto_dir / "Detection_pb2.py")
ImageAnnotations_pb2  = load_proto("ImageAnnotations_pb2",  proto_dir / "ImageAnnotations_pb2.py")
Mavlink_pb2           = load_proto("Mavlink_pb2",           proto_dir / "Mavlink_pb2.py")
System_pb2            = load_proto("System_pb2",            proto_dir / "System_pb2.py")

# ---------------------------------------
#  Now we can safely load Target_pb2
# ---------------------------------------
Target_pb2 = load_proto("Target_pb2", proto_dir / "Target_pb2.py")

TargetDetection = Target_pb2.TargetDetection

# ---------------------------------------------------------------------
#   FIX TARGET DETECTION MESSAGE (swap XY + rename fields + rad→deg)
# ---------------------------------------------------------------------
def fix_target_detection(msg):
    changed = False

    # -------------------------
    # 1. Extract old values
    # -------------------------
    old_raw_x  = getattr(msg, "bbox_offset_x_px", None)
    old_raw_y  = getattr(msg, "bbox_offset_y_px", None)
    old_filt_x = getattr(msg, "bbox_center_x_px", None)
    old_filt_y = getattr(msg, "bbox_center_y_px", None)

    # -------------------------
    # 2. Assign new names (SWAPPED)
    # -------------------------
    # raw (swap)
    if old_raw_x is not None and old_raw_y is not None:
        msg.bbox_center_offset_x_px_raw = old_raw_y
        msg.bbox_center_offset_y_px_raw = old_raw_x
        changed = True

    # filtered (swap)
    if old_filt_x is not None and old_filt_y is not None:
        msg.bbox_center_offset_x_px_filt = old_filt_y
        msg.bbox_center_offset_y_px_filt = old_filt_x
        changed = True

    # -------------------------
    # 3. Remove old fields by resetting to 0
    # -------------------------
    if hasattr(msg, "bbox_offset_x_px"):
        msg.bbox_offset_x_px = 0.0
    if hasattr(msg, "bbox_offset_y_px"):
        msg.bbox_offset_y_px = 0.0
    if hasattr(msg, "bbox_center_x_px"):
        msg.bbox_center_x_px = 0.0
    if hasattr(msg, "bbox_center_y_px"):
        msg.bbox_center_y_px = 0.0

    # -------------------------
    # 4. Rename + convert angles
    # -------------------------
    if hasattr(msg, "delta_angle_deg"):
        msg.delta_angle = msg.delta_angle_deg * DEG_PER_RAD
        msg.delta_angle_deg = 0.0
        changed = True

    if hasattr(msg, "camera_tilt_deg"):
        msg.camera_tilt_angle = msg.camera_tilt_deg * DEG_PER_RAD
        msg.camera_tilt_deg = 0.0
        changed = True

    return changed


# ---------------------------------------------------------------------
#   FIX MAV KINEMATICS MESSAGE (rad→deg)
# ---------------------------------------------------------------------
def fix_mav_kinematics(msg):
    angles = [
        "roll", "pitch", "yaw",
        "rollspeed", "pitchspeed", "yawspeed",
        "roll_rate_actual", "pitch_rate_actual", "yaw_rate_actual",
    ]
    changed = False
    for f in angles:
        if hasattr(msg, f):
            setattr(msg, f, getattr(msg, f) * DEG_PER_RAD)
            changed = True
    return changed


def copy_common_fields(old_msg, new_msg):
    """
    Copy old_msg → new_msg
      • primitive fields use direct assign
      • submessages use CopyFrom()
    Only copies fields existing in BOTH messages.
    """
    for field in old_msg.DESCRIPTOR.fields:
        name = field.name

        if not hasattr(new_msg, name):
            continue  # skip removed/renamed fields

        old_value = getattr(old_msg, name)

        if field.type == field.TYPE_MESSAGE:
            # Create a NEW submessage of the correct (new) type
            sub_old = old_value
            sub_new = getattr(new_msg, name)

            # Copy each scalar field
            for subfield in sub_old.DESCRIPTOR.fields:
                subname = subfield.name
                if hasattr(sub_new, subname):
                    val = getattr(sub_old, subname)
                    setattr(sub_new, subname, val)

        else:
            # scalar field
            setattr(new_msg, name, old_value)


# ---------------------------------------------------------------------
#   HANDLE EACH MESSAGE
# ---------------------------------------------------------------------
def process_message(msg_name, proto_msg):
    if msg_name == TARGET_DET_MSG:
        return fix_target_detection(proto_msg)
    if msg_name == MAV_KIN_MSG:
        return fix_mav_kinematics(proto_msg)
    return False


# ---------------------------------------------------------------------
#   FIX A SINGLE MCAP FILE
# ---------------------------------------------------------------------
def fix_single_mcap(input_path: Path):
    output_path = input_path.with_name(input_path.stem + "_fixed_deg_swapped.mcap")
    print(f"🔧 Fixing {input_path} → {output_path}")

    with open(input_path, "rb") as f_in, open(output_path, "wb") as f_out:
        reader  = make_reader(f_in)
        writer  = Writer(f_out)
        decoder = Decoder()

        writer.start()
        summary  = reader.get_summary()

        schemas  = summary.schemas
        channels = summary.channels

        # Register schemas as-is
        schema_map = {}
        for sid, schema in schemas.items():
            new_id = writer.register_schema(
                name=schema.name,
                encoding=schema.encoding,
                data=schema.data,
            )
            schema_map[sid] = new_id

        # Register channels
        channel_map = {}
        for cid, ch in channels.items():
            new_ch = writer.register_channel(
                topic=ch.topic,
                message_encoding=ch.message_encoding,
                schema_id=schema_map[ch.schema_id],
                metadata=ch.metadata or {},
            )
            channel_map[cid] = new_ch

        modified = 0

        # Process messages
        for schema, channel, message in reader.iter_messages():
            data = message.data

            if schema is not None:
                msg_name = schema.name

                # -----------------------------------------------------
                # Handle TARGET DETECTION (needs schema upgrade + fixes)
                # -----------------------------------------------------
                if msg_name == TARGET_DET_MSG:
                    # decode old schema message
                    old_msg = decoder.decode(schema, message)

                    # create new message using updated schema
                    new_msg = TargetDetection()

                    # copy shared fields (safe method)
                    copy_common_fields(old_msg, new_msg)

                    # apply swap + rename + rad→deg changes
                    fix_target_detection(new_msg)

                    # serialize new message
                    data = new_msg.SerializeToString()
                    modified += 1


                # -----------------------------------------------------
                # Handle MAV KINEMATICS (simple rad→deg on old schema)
                # -----------------------------------------------------
                elif msg_name == MAV_KIN_MSG:
                    old_msg = decoder.decode(schema, message)
                    changed = fix_mav_kinematics(old_msg)
                    if changed:
                        data = old_msg.SerializeToString()
                        modified += 1

            # Write (original or modified) message
            writer.add_message(
                channel_id=channel_map[channel.id],
                log_time=message.log_time,
                publish_time=message.publish_time,
                data=data,
                sequence=message.sequence,
            )


        writer.finish()

    print(f"✅ Updated {modified} messages → {output_path}\n")


# ---------------------------------------------------------------------
#   FIX ALL MCAPS IN A DIRECTORY
# ---------------------------------------------------------------------
def fix_all_in_directory(root: Path):
    print(f"\n🚀 Scanning recursively under: {root}\n")

    files = list(root.rglob("*.mcap"))
    if not files:
        print("❌ No MCAP files found.")
        return

    print(f"📦 Found {len(files)} MCAP files.\n")

    for f in files:
        fix_single_mcap(f)

    print("\n🎉 All files processed.\n")


# ---------------------------------------------------------------------
#   MAIN
# ---------------------------------------------------------------------
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 modify_mcap_vars.py /path/to/folder")
        sys.exit(1)

    folder = Path(sys.argv[1]).expanduser().resolve()
    fix_all_in_directory(folder)
