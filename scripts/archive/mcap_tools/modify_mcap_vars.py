#!/usr/bin/env python3
import sys
import math
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_protobuf.decoder import Decoder

TARGET_MESSAGE_FULLNAME = "os.logger.TargetDetection"

DEG_PER_RAD = 180.0 / math.pi


def fix_angles(msg):
    """Convert rad -> deg for incorrectly-logged angle fields."""
    if hasattr(msg, "delta_angle_deg"):
        msg.delta_angle_deg = msg.delta_angle_deg * DEG_PER_RAD

    if hasattr(msg, "camera_tilt_deg"):
        msg.camera_tilt_deg = msg.camera_tilt_deg * DEG_PER_RAD


def fix_single_mcap(input_path: Path):
    output_path = input_path.with_name(input_path.stem + "2.mcap")

    print(f"🔧 Fixing {input_path} → {output_path}")

    with open(input_path, "rb") as f_in, open(output_path, "wb") as f_out:
        reader = make_reader(f_in)
        writer = Writer(f_out)
        decoder = Decoder()

        writer.start()

        summary = reader.get_summary()
        schemas = summary.schemas
        channels = summary.channels

        print("  Schemas in file:")
        for sid, schema in schemas.items():
            print(f"    id={sid} name={schema.name}")

        # --- register schemas + channels for new file ---
        schema_id_map = {}
        channel_id_map = {}

        for schema_id, schema in schemas.items():
            new_schema_id = writer.register_schema(
                name=schema.name,
                encoding=schema.encoding,
                data=schema.data,
            )
            schema_id_map[schema_id] = new_schema_id

        for channel_id, channel in channels.items():
            new_channel_id = writer.register_channel(
                topic=channel.topic,
                message_encoding=channel.message_encoding,
                schema_id=schema_id_map.get(channel.schema_id, 0),
                metadata=channel.metadata or {},
            )
            channel_id_map[channel_id] = new_channel_id

        modified = 0

        # --- iterate messages ---
        for schema, channel, message in reader.iter_messages():
            data = message.data

            if schema is not None and schema.name == TARGET_MESSAGE_FULLNAME:
                proto_msg = decoder.decode(schema, message)

                before_delta = getattr(proto_msg, "delta_angle_deg", None)
                before_tilt = getattr(proto_msg, "camera_tilt_deg", None)

                fix_angles(proto_msg)

                after_delta = getattr(proto_msg, "delta_angle_deg", None)
                after_tilt = getattr(proto_msg, "camera_tilt_deg", None)

                if modified < 5:
                    print(
                        f"    angles rad→deg: "
                        f"delta {before_delta} → {after_delta}, "
                        f"tilt {before_tilt} → {after_tilt}"
                    )

                modified += 1
                data = proto_msg.SerializeToString()

            writer.add_message(
                channel_id=channel_id_map[channel.id],
                log_time=message.log_time,
                publish_time=message.publish_time,
                data=data,
                sequence=message.sequence,
            )

        writer.finish()

    print(f"✅ Wrote: {output_path} (modified {modified} messages)\n")


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


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 rename_mcap_vars2.py /path/to/folder")
        sys.exit(1)

    folder = Path(sys.argv[1]).expanduser().resolve()
    fix_all_in_directory(folder)
