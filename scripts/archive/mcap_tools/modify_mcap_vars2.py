#!/usr/bin/env python3
import sys
import math
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_protobuf.decoder import Decoder

TARGET_DET_MSG = "os.logger.TargetDetection"
MAV_KIN_MSG   = "os.logger.MavKinematics"

DEG_PER_RAD = 180.0 / math.pi


# -------------------------------------------------------
#   Helper: Convert radians → degrees for fields
# -------------------------------------------------------
def convert_rad_to_deg_fields(msg, fields):
    for f in fields:
        if hasattr(msg, f):
            val = getattr(msg, f)
            setattr(msg, f, val * DEG_PER_RAD)


# -------------------------------------------------------
#   Modify each message
# -------------------------------------------------------
def fix_message(schema_name, proto_msg):
    if schema_name == TARGET_DET_MSG:
        convert_rad_to_deg_fields(
            proto_msg,
            ["delta_angle_deg", "camera_tilt_deg"]
        )
        return True

    if schema_name == MAV_KIN_MSG:
        convert_rad_to_deg_fields(
            proto_msg,
            [
                "roll", "pitch", "yaw",
                "rollspeed", "pitchspeed", "yawspeed",
                "roll_rate_actual", "pitch_rate_actual", "yaw_rate_actual"
            ]
        )
        return True

    return False  # unchanged


# -------------------------------------------------------
#   Fix a single MCAP
# -------------------------------------------------------
def fix_single_mcap(input_path: Path):
    output_path = input_path.with_name(input_path.stem + "_deg.mcap")

    print(f"🔧 Fixing {input_path} → {output_path}")

    with open(input_path, "rb") as f_in, open(output_path, "wb") as f_out:

        reader = make_reader(f_in)
        writer = Writer(f_out)
        decoder = Decoder()

        writer.start()
        summary  = reader.get_summary()
        schemas  = summary.schemas
        channels = summary.channels

        print("  Schemas in file:")
        for sid, schema in schemas.items():
            print(f"    id={sid} name={schema.name}")

        # Register schemas
        schema_id_map = {
            sid: writer.register_schema(
                name=schema.name,
                encoding=schema.encoding,
                data=schema.data,
            )
            for sid, schema in schemas.items()
        }

        # Register channels
        channel_id_map = {
            cid: writer.register_channel(
                topic=channel.topic,
                message_encoding=channel.message_encoding,
                schema_id=schema_id_map.get(channel.schema_id, 0),
                metadata=channel.metadata or {},
            )
            for cid, channel in channels.items()
        }

        modified = 0

        # Iterate messages
        for schema, channel, message in reader.iter_messages():
            data = message.data
            if schema is not None:
                msg_name = schema.name

                if msg_name in (TARGET_DET_MSG, MAV_KIN_MSG):
                    proto_msg = decoder.decode(schema, message)

                    before = {}
                    after = {}

                    # record initial values for debug (first few only)
                    if modified < 5:
                        tracked_fields = [
                            "delta_angle_deg",
                            "camera_tilt_deg",
                            "roll", "pitch", "yaw",
                            "rollspeed", "pitchspeed", "yawspeed",
                            "roll_rate_actual", "pitch_rate_actual", "yaw_rate_actual",
                        ]
                        for f in tracked_fields:
                            if hasattr(proto_msg, f):
                                before[f] = getattr(proto_msg, f)

                    changed = fix_message(msg_name, proto_msg)

                    if changed:
                        modified += 1
                        data = proto_msg.SerializeToString()

                        if modified <= 5:
                            tracked_fields = before.keys()
                            for f in tracked_fields:
                                after[f] = getattr(proto_msg, f)

                            print(f"    [{msg_name}] rad→deg:")
                            for f in tracked_fields:
                                print(f"      {f}: {before[f]} → {after[f]}")

            # Write message
            writer.add_message(
                channel_id=channel_id_map[channel.id],
                log_time=message.log_time,
                publish_time=message.publish_time,
                data=data,
                sequence=message.sequence,
            )

        writer.finish()

    print(f"✅ Wrote: {output_path}  (modified {modified} messages)\n")


# -------------------------------------------------------
#   Fix a directory recursively
# -------------------------------------------------------
def fix_all_in_directory(root: Path):
    print(f"\n🚀 Scanning recursively under: {root}\n")

    mcap_files = list(root.rglob("*.mcap"))
    if not mcap_files:
        print("❌ No MCAP files found.")
        return

    print(f"📦 Found {len(mcap_files)} MCAP files.\n")

    for f in mcap_files:
        fix_single_mcap(f)

    print("\n🎉 All files processed.\n")


# -------------------------------------------------------
#   Entry point
# -------------------------------------------------------
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 modify_mcap_vars.py /path/to/folder")
        sys.exit(1)

    folder = Path(sys.argv[1]).expanduser().resolve()
    fix_all_in_directory(folder)
