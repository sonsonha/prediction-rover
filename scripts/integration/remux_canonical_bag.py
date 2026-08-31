#!/usr/bin/env python3
"""Filter canonical bag to Prediction INPUT topics only (preserve timestamps)."""

from __future__ import annotations

import argparse
import os
import shutil
import sys
from pathlib import Path

from rosbag2_py import ConverterOptions, SequentialReader, SequentialWriter, StorageOptions, TopicMetadata

INPUT_TOPICS = {
    "/trajectory",
    "/tracked_objects",
    "/geometry",
    "/rover/state",
}


def remux(src: Path, dst: Path) -> None:
    if dst.exists():
        shutil.rmtree(dst)
    dst.parent.mkdir(parents=True, exist_ok=True)

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(src), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    topic_types: dict[str, str] = {}
    for meta in reader.get_all_topics_and_types():
        if meta.name in INPUT_TOPICS:
            topic_types[meta.name] = meta.type

    missing = INPUT_TOPICS - set(topic_types)
    if missing:
        raise SystemExit(f"missing topics in source bag: {sorted(missing)}")

    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(dst), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    for name, typ in sorted(topic_types.items()):
        writer.create_topic(TopicMetadata(name=name, type=typ, serialization_format="cdr"))

    counts: dict[str, int] = {t: 0 for t in INPUT_TOPICS}
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic not in INPUT_TOPICS:
            continue
        writer.write(topic, data, timestamp)
        counts[topic] += 1

    total = sum(counts.values())
    if total == 0:
        raise SystemExit("remux produced zero messages")
    _fix_bag_filenames(dst)
    print(f"remux OK: {total} messages -> {dst}")
    for topic in sorted(counts):
        print(f"  {topic}: {counts[topic]}")


def _fix_bag_filenames(bag_dir: Path) -> None:
    """Normalize db3 filename + metadata after rosbag2_py write."""
    db3_files = sorted(bag_dir.glob("*.db3"))
    if not db3_files:
        raise SystemExit(f"no db3 in {bag_dir}")
    old = db3_files[0]
    canonical = bag_dir / f"{bag_dir.name}_0.db3"
    if old != canonical:
        old_name = old.name
        old.rename(canonical)
        meta = bag_dir / "metadata.yaml"
        text = meta.read_text()
        text = text.replace(old_name, canonical.name)
        meta.write_text(text)


def _default_bags_dir() -> Path:
    workspace = Path(os.environ.get("ROVER_WORKSPACE", "/data/rover_workspace"))
    return workspace / "prediction" / "bags"


def main() -> int:
    bags = _default_bags_dir()
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--src",
        type=Path,
        default=bags / "session_0924_pipe_prediction_inputs.with_predict_output.bak",
    )
    parser.add_argument(
        "--dst",
        type=Path,
        default=bags / "session_0924_pipe_prediction_inputs",
    )
    args = parser.parse_args()
    if not (args.src / "metadata.yaml").exists():
        print(f"FAIL: source bag missing: {args.src}", file=sys.stderr)
        return 1
    remux(args.src, args.dst)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
