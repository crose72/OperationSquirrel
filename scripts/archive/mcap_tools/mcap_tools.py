#!/usr/bin/env python3
import argparse
from pathlib import Path
from mcap.reader import make_reader
from mcap.writer import Writer
from mcap.records import Message
import importlib


class McapHelper:
    def __init__(self, input_path: str):
        self.input_path = Path(input_path)
        if not self.input_path.exists():
            raise FileNotFoundError(f"MCAP file not found: {self.input_path}")

        self.channels = {}
        self.schemas = {}
        self.messages_by_topic = {}

    # ---------------------------------------------------------
    # Load MCAP (MCAP >=1.0 API)
    # ---------------------------------------------------------
    def load(self):
        with self.input_path.open("rb") as f:
            reader = make_reader(f)

            # Build channel lookup table
            # (schemas are not needed for swapping)
            self.channels = {}

            # Load all messages
            for schema, channel, msg in reader.iter_messages():
                topic = channel.topic

                # record channel by id
                self.channels[channel.id] = channel

                if topic not in self.messages_by_topic:
                    self.messages_by_topic[topic] = []

                # store EXACT tuple from reader
                self.messages_by_topic[topic].append((schema, channel, msg))

        print("Loaded MCAP topics:")
        for t in self.messages_by_topic:
            print("  -", t)

    # ---------------------------------------------------------
    # Swap two fields inside protobuf messages
    # ---------------------------------------------------------
    def swap_fields_in_topic(self, topic, fieldA, fieldB, proto_cls):
        msgs = self.messages_by_topic[topic]
        new_msgs = []

        for schema, channel, msg in msgs:
            pb = proto_cls()
            pb.ParseFromString(msg.data)

            # swap
            temp = getattr(pb, fieldA)
            setattr(pb, fieldA, getattr(pb, fieldB))
            setattr(pb, fieldB, temp)

            new_msg = Message(
                channel_id=channel.id,
                log_time=msg.log_time,
                publish_time=msg.publish_time,
                data=pb.SerializeToString()
            )

            new_msgs.append((schema, channel, new_msg))

        self.messages_by_topic[topic] = new_msgs

    # ---------------------------------------------------------
    # Save MCAP
    # ---------------------------------------------------------
    def save(self, output_path: str):
        output_path = Path(output_path)
        with output_path.open("wb") as f:
            writer = Writer(f)

            # Write only messages, no schemas/channels, because this Writer does not support them
            for topic, msgs in self.messages_by_topic.items():
                for schema, channel, msg in msgs:
                    writer.write_message(msg)

            writer.finish()

        print(f"Saved modified MCAP → {output_path}")

# ==================================================================
# CLI
# ==================================================================
def parse_args():
    parser = argparse.ArgumentParser(description="MCAP manipulation tool")

    parser.add_argument("input", help="Input MCAP file")
    parser.add_argument("output", help="Output MCAP file")

    parser.add_argument("--swap-fields", nargs=3,
                        metavar=("TOPIC", "FIELD_A", "FIELD_B"),
                        help="Swap two protobuf fields inside the same topic")

    parser.add_argument("--proto", type=str,
                        help="Protobuf class for deserialization, format module:Class")

    return parser.parse_args()


def load_proto_class(spec: str):
    module_name, class_name = spec.split(":")
    module = importlib.import_module(module_name)
    return getattr(module, class_name)


def main():
    args = parse_args()

    helper = McapHelper(args.input)
    helper.load()

    if args.swap_fields:
        if not args.proto:
            raise ValueError("You must provide --proto when using --swap-fields")

        proto_cls = load_proto_class(args.proto)
        topic, fieldA, fieldB = args.swap_fields
        helper.swap_fields_in_topic(topic, fieldA, fieldB, proto_cls)

    helper.save(args.output)


if __name__ == "__main__":
    main()
