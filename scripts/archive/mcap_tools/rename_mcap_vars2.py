#!/usr/bin/env python3
import sys
from pathlib import Path

from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_protobuf.decoder import Decoder

TARGET_MESSAGE_FULLNAME = "os.logger.TargetDetection"  # ✅ this matches your schema


def fix_offsets(msg):
    orig_x = msg.bbox_offset_x_px
    orig_y = msg.bbox_offset_y_px
    orig_x_filt = msg.bbox_center_x_px
    orig_y_filt = msg.bbox_center_y_px
    msg.bbox_offset_x_px = orig_y
    msg.bbox_offset_y_px = orig_x
    msg.bbox_center_x_px = orig_y_filt
    msg.bbox_center_y_px = orig_x_filt


def fix_single_mcap(input_path: Path):
    output_path = input_path.with_name(input_path.stem + "_fixed.mcap")

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

        schema_id_map = {}
        channel_id_map = {}

        # Copy schemas
        for schema_id, schema in schemas.items():
            new_schema_id = writer.register_schema(
                name=schema.name,
                encoding=schema.encoding,
                data=schema.data,
            )
            schema_id_map[schema_id] = new_schema_id

        # Copy channels
        for channel_id, channel in channels.items():
            new_channel_id = writer.register_channel(
                topic=channel.topic,
                message_encoding=channel.message_encoding,
                schema_id=schema_id_map.get(channel.schema_id, 0),
                metadata=channel.metadata or {},
            )
            channel_id_map[channel_id] = new_channel_id

        modified = 0

        for schema, channel, message in reader.iter_messages():
            data = message.data

            if schema is not None and schema.name == TARGET_MESSAGE_FULLNAME:
                # ✅ no message_encoding filter anymore
                proto_msg = decoder.decode(schema, message)

                before_x = proto_msg.bbox_offset_x_px
                before_y = proto_msg.bbox_offset_y_px

                fix_offsets(proto_msg)

                after_x = proto_msg.bbox_offset_x_px
                after_y = proto_msg.bbox_offset_y_px

                if modified < 5:  # only spam a few for sanity
                    print(
                        f"    swapped msg on topic {channel.topic}: "
                        f"x {before_x} -> {after_x}, "
                        f"y {before_y} -> {after_y}"
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
