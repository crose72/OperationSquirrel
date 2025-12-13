#!/usr/bin/env python3
import sys
import os
import glob

from mcap.reader import make_reader
from mcap.writer import Writer

from google.protobuf.descriptor_pb2 import FileDescriptorSet
from google.protobuf.message import DecodeError

# ---------------------------------------------
# CONFIG
# ---------------------------------------------
TOPIC = "/control/output"

# Old → new field names inside ControlOutput
RENAME_MAP = {
    "cmd_yawrate_rps": "cmd_yaw_rad",
    "id_target_too_close": "is_target_too_close",
}

print(f"🔧 Updating schema for topic {TOPIC}:")
for old, new in RENAME_MAP.items():
    print(f"   {old} → {new}")


def patch_schema(schema) -> bytes:
    """
    Patch the protobuf FileDescriptorSet inside schema.data
    to rename fields in the ControlOutput message.

    We:
      - parse schema.data as FileDescriptorSet
      - locate message_type == 'ControlOutput'
      - rename fields using RENAME_MAP
      - re-serialize the descriptor set

    If parsing fails, we leave schema.data untouched.
    """
    fds = FileDescriptorSet()
    try:
        fds.ParseFromString(schema.data)
    except DecodeError:
        # Not a FileDescriptorSet we recognize; don't touch it.
        print("⚠️  Could not parse schema.data as FileDescriptorSet, leaving it unchanged.")
        return schema.data

    changed = False

    for file_proto in fds.file:
        for msg in file_proto.message_type:
            if msg.name == "ControlOutput":
                for field in msg.field:
                    if field.name in RENAME_MAP:
                        old_name = field.name
                        new_name = RENAME_MAP[old_name]
                        field.name = new_name
                        changed = True
                        print(f"   ✏️  Renamed field in descriptor: {old_name} → {new_name}")

    if not changed:
        # Nothing to change, return original data
        return schema.data

    return fds.SerializeToString()


def process_file(path):
    out_path = path.replace(".mcap", "_mod.mcap")

    print(f"\n📂 Reading:  {path}")
    print(f"💾 Writing: {out_path}")

    with open(path, "rb") as f_in, open(out_path, "wb") as f_out:
        reader = make_reader(f_in)
        writer = Writer(f_out)
        writer.start()

        schema_id_map = {}
        channel_id_map = {}

        for schema, channel, message in reader.iter_messages():
            # -------------------------------
            # Register schema (patching only for /control/output)
            # -------------------------------
            if schema:
                if schema.id not in schema_id_map:
                    if channel.topic == TOPIC:
                        new_schema_data = patch_schema(schema)
                    else:
                        new_schema_data = schema.data

                    new_schema_id = writer.register_schema(
                        name=schema.name,
                        encoding=schema.encoding,
                        data=new_schema_data,
                    )
                    schema_id_map[schema.id] = new_schema_id

                new_schema_id = schema_id_map[schema.id]
            else:
                new_schema_id = None

            # -------------------------------
            # Register channel
            # -------------------------------
            if channel.id not in channel_id_map:
                new_channel_id = writer.register_channel(
                    schema_id=new_schema_id,
                    topic=channel.topic,
                    message_encoding=channel.message_encoding,
                )
                channel_id_map[channel.id] = new_channel_id

            out_channel_id = channel_id_map[channel.id]

            # -------------------------------
            # IMPORTANT: do NOT touch data bytes
            # -------------------------------
            writer.add_message(
                channel_id=out_channel_id,
                log_time=message.log_time,
                publish_time=message.publish_time,
                data=message.data,
            )

        writer.finish()
        print(f"✅ Finished: {out_path}")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 rename_mcap_signal.py <folder> <pattern>")
        sys.exit(1)

    folder = sys.argv[1]
    pattern = sys.argv[2]

    files = glob.glob(os.path.join(folder, pattern))
    if not files:
        print("❌ No matching MCAP files found.")
        sys.exit(1)

    for f in files:
        process_file(f)

    print("\n🎉 All files processed successfully.")
