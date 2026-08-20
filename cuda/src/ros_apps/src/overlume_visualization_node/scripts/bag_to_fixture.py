#!/usr/bin/env python3
"""bag_to_fixture.py -- dump N messages from one bag topic to YAML test
fixtures (Epic 2 Task 1 / VM-020, "Fixture strategy").

Each message becomes one `<topic-slug>_<n>.yaml` via
rosidl_runtime_py.message_to_yaml -- human-readable, diffable, and read
back by test/fixture_msgs.{hpp,cpp}.

The two map topics are NOT "a few KB each" unfiltered: one
/hd_map_local_elements message (701 markers) is 0.66 MB, and the single
latched /sim/hd_map/markers message (3725 markers) is 3.88 MB. So this
script filters (--max-markers-per-ns, --max-ns-per-prefix) and refuses to
write a fixture larger than 256 KB unless --allow-big is passed -- a
multi-MB fixture must be a deliberate choice, not a review-time surprise.
"""
import argparse
import os
import re
import sys
from collections import OrderedDict

import rclpy  # noqa: F401  -- initializes message type support as a side effect
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message

MAX_FIXTURE_BYTES = 256 * 1024

# 'centerline_0' -> 'centerline_' ; 'crosswalks' (no numeric suffix) -> itself.
_NUMERIC_SUFFIX_RE = re.compile(r"^(.*?)(\d+)$")


def _namespace_family(ns: str) -> str:
    m = _NUMERIC_SUFFIX_RE.match(ns)
    return m.group(1) if m else ns


def _filter_marker_array(msg, max_markers_per_ns, max_ns_per_prefix):
    """Keeps at most `max_markers_per_ns` markers per distinct namespace and
    at most `max_ns_per_prefix` distinct namespaces per numeric-suffix
    family, preserving first-seen order of both markers and namespaces."""
    by_ns = OrderedDict()
    for m in msg.markers:
        by_ns.setdefault(m.ns, []).append(m)

    namespaces = list(by_ns.keys())
    if max_ns_per_prefix is not None:
        family_count = {}
        kept_ns = []
        for ns in namespaces:
            fam = _namespace_family(ns)
            if family_count.get(fam, 0) < max_ns_per_prefix:
                kept_ns.append(ns)
                family_count[fam] = family_count.get(fam, 0) + 1
        namespaces = kept_ns

    kept_markers = []
    for ns in namespaces:
        markers = by_ns[ns]
        if max_markers_per_ns is not None:
            markers = markers[:max_markers_per_ns]
        kept_markers.extend(markers)

    msg.markers = kept_markers
    return msg


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", help="path to the rosbag2 directory (contains metadata.yaml)")
    parser.add_argument("topic", help="topic name, e.g. /hd_map_local_elements")
    parser.add_argument("--count", type=int, default=1, help="number of messages to dump")
    parser.add_argument("--out", default=".", help="output directory")
    parser.add_argument("--max-markers-per-ns", type=int, default=None,
                         help="MarkerArray only: keep at most N markers per namespace")
    parser.add_argument("--max-ns-per-prefix", type=int, default=None,
                         help="MarkerArray only: keep at most K namespaces per numeric-suffix family")
    parser.add_argument("--allow-big", action="store_true",
                         help="write a fixture bigger than 256 KB instead of refusing")
    args = parser.parse_args()

    storage_options = rosbag2_py.StorageOptions(uri=args.bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.topic not in type_by_topic:
        print(f"error: topic '{args.topic}' not found in bag '{args.bag_path}'", file=sys.stderr)
        return 1
    msg_type = get_message(type_by_topic[args.topic])

    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    os.makedirs(args.out, exist_ok=True)
    slug = args.topic.strip("/").replace("/", "_")

    written = 0
    while reader.has_next() and written < args.count:
        _topic, data, _t = reader.read_next()
        msg = deserialize_message(data, msg_type)

        if hasattr(msg, "markers") and (args.max_markers_per_ns is not None or
                                         args.max_ns_per_prefix is not None):
            msg = _filter_marker_array(msg, args.max_markers_per_ns, args.max_ns_per_prefix)

        text = message_to_yaml(msg)
        size = len(text.encode("utf-8"))
        if size > MAX_FIXTURE_BYTES and not args.allow_big:
            print(f"error: fixture would be {size} bytes (> {MAX_FIXTURE_BYTES} B guard). "
                  "Pass --max-markers-per-ns/--max-ns-per-prefix to shrink it, "
                  "or --allow-big to override.", file=sys.stderr)
            return 1

        out_path = os.path.join(args.out, f"{slug}_{written}.yaml")
        with open(out_path, "w") as f:
            f.write(text)
        print(f"wrote {out_path} ({size} bytes)")
        written += 1

    if written == 0:
        print(f"error: no messages found on topic '{args.topic}' in this bag", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
