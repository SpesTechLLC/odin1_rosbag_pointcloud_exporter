#!/usr/bin/env python3
"""Export and accumulate Odin1 colored PointCloud2 data to PCD or PLY.

The exporter reads the bag offline.  Point data is never accumulated in RAM.
When voxel filtering is enabled, intermediate voxel records are hash-partitioned
on disk and only one partition is reduced in memory at a time.
"""

from __future__ import annotations

import argparse
import bisect
import collections
import math
import os
from pathlib import Path
import shutil
import struct
import sys
import tempfile
import time
from typing import Dict, Iterable, List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

try:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs.msg import PointField
except ImportError as exc:  # pragma: no cover - depends on the ROS environment
    raise SystemExit(
        "ROS 2 Python modules were not found. Source /opt/ros/<distro>/setup.bash first."
    ) from exc


PCD_POINT_DTYPE = np.dtype(
    [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")]
)
PLY_POINT_DTYPE = np.dtype(
    [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("red", "u1"),
        ("green", "u1"),
        ("blue", "u1"),
    ]
)
VOXEL_RECORD_DTYPE = np.dtype(
    [
        ("ix", "<i8"),
        ("iy", "<i8"),
        ("iz", "<i8"),
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("rgb", "<u4"),
    ]
)
VOXEL_KEY_DTYPE = np.dtype([("ix", "<i8"), ("iy", "<i8"), ("iz", "<i8")])


class TfSample(NamedTuple):
    stamp_ns: int
    translation: np.ndarray
    quaternion: np.ndarray  # x, y, z, w


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def normalize_frame(frame: str) -> str:
    return frame.lstrip("/")


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    norm = float(np.dot(q, q))
    if norm < 1e-20:
        raise ValueError("TF contains a zero-length quaternion")
    q *= math.sqrt(2.0 / norm)
    outer = np.outer(q, q)
    return np.array(
        [
            [1.0 - outer[1, 1] - outer[2, 2], outer[0, 1] - outer[2, 3], outer[0, 2] + outer[1, 3]],
            [outer[0, 1] + outer[2, 3], 1.0 - outer[0, 0] - outer[2, 2], outer[1, 2] - outer[0, 3]],
            [outer[0, 2] - outer[1, 3], outer[1, 2] + outer[0, 3], 1.0 - outer[0, 0] - outer[1, 1]],
        ],
        dtype=np.float64,
    )


def slerp(q0: np.ndarray, q1: np.ndarray, ratio: float) -> np.ndarray:
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        result = q0 + ratio * (q1 - q0)
        return result / np.linalg.norm(result)
    theta = math.acos(max(-1.0, min(1.0, dot)))
    sin_theta = math.sin(theta)
    return (
        math.sin((1.0 - ratio) * theta) / sin_theta * q0
        + math.sin(ratio * theta) / sin_theta * q1
    )


class TfArchive:
    """Small offline TF archive with graph lookup and linear/slerp interpolation."""

    def __init__(self) -> None:
        self.dynamic: Dict[Tuple[str, str], List[TfSample]] = collections.defaultdict(list)
        self.static: Dict[Tuple[str, str], TfSample] = {}
        self._stamps: Dict[Tuple[str, str], List[int]] = {}

    def add_message(self, msg, is_static: bool) -> None:
        for transform in msg.transforms:
            parent = normalize_frame(transform.header.frame_id)
            child = normalize_frame(transform.child_frame_id)
            if not parent or not child:
                continue
            tr = transform.transform.translation
            rot = transform.transform.rotation
            sample = TfSample(
                stamp_ns(transform.header.stamp),
                np.array([tr.x, tr.y, tr.z], dtype=np.float64),
                np.array([rot.x, rot.y, rot.z, rot.w], dtype=np.float64),
            )
            if is_static:
                self.static[(parent, child)] = sample
            else:
                self.dynamic[(parent, child)].append(sample)

    def finalize(self) -> None:
        for edge, samples in self.dynamic.items():
            samples.sort(key=lambda item: item.stamp_ns)
            self._stamps[edge] = [item.stamp_ns for item in samples]

    def _sample_edge(self, edge: Tuple[str, str], at_ns: int, max_gap_ns: int) -> np.ndarray:
        if edge in self.static:
            sample = self.static[edge]
            translation, quaternion = sample.translation, sample.quaternion
        else:
            samples = self.dynamic.get(edge)
            if not samples:
                raise KeyError(edge)
            times = self._stamps[edge]
            right = bisect.bisect_left(times, at_ns)
            if right == 0:
                nearest_gap = times[0] - at_ns
                if nearest_gap > max_gap_ns:
                    raise LookupError(f"TF {edge[0]} <- {edge[1]} is {nearest_gap / 1e9:.3f}s too new")
                translation, quaternion = samples[0].translation, samples[0].quaternion
            elif right == len(samples):
                nearest_gap = at_ns - times[-1]
                if nearest_gap > max_gap_ns:
                    raise LookupError(f"TF {edge[0]} <- {edge[1]} is {nearest_gap / 1e9:.3f}s too old")
                translation, quaternion = samples[-1].translation, samples[-1].quaternion
            else:
                before, after = samples[right - 1], samples[right]
                interval = after.stamp_ns - before.stamp_ns
                ratio = 0.0 if interval == 0 else (at_ns - before.stamp_ns) / interval
                translation = before.translation + ratio * (after.translation - before.translation)
                quaternion = slerp(before.quaternion, after.quaternion, ratio)
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = quaternion_to_matrix(quaternion)
        matrix[:3, 3] = translation
        return matrix

    def lookup(self, target: str, source: str, at_ns: int, max_gap_ns: int) -> np.ndarray:
        target, source = normalize_frame(target), normalize_frame(source)
        if target == source:
            return np.eye(4, dtype=np.float64)

        edges = set(self.dynamic) | set(self.static)
        graph: Dict[str, List[Tuple[str, Tuple[str, str], bool]]] = collections.defaultdict(list)
        for parent, child in edges:
            graph[child].append((parent, (parent, child), False))
            graph[parent].append((child, (parent, child), True))

        queue = collections.deque([(source, np.eye(4, dtype=np.float64))])
        visited = {source}
        while queue:
            current, current_from_source = queue.popleft()
            for next_frame, edge, invert in graph.get(current, []):
                if next_frame in visited:
                    continue
                edge_matrix = self._sample_edge(edge, at_ns, max_gap_ns)
                next_from_current = np.linalg.inv(edge_matrix) if invert else edge_matrix
                next_from_source = next_from_current @ current_from_source
                if next_frame == target:
                    return next_from_source
                visited.add(next_frame)
                queue.append((next_frame, next_from_source))
        raise LookupError(f"no TF path from '{source}' to '{target}'")


def open_reader(bag_path: Path, topics: Sequence[str]) -> Tuple[SequentialReader, Dict[str, str]]:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id=""), ConverterOptions("", ""))
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [topic for topic in topics if topic not in topic_types]
    if missing:
        raise RuntimeError(f"topic(s) not found in bag: {', '.join(missing)}")
    reader.set_filter(StorageFilter(topics=list(topics)))
    return reader, topic_types


def load_tf_archive(bag_path: Path) -> TfArchive:
    probe = SequentialReader()
    probe.open(StorageOptions(uri=str(bag_path), storage_id=""), ConverterOptions("", ""))
    types = {item.name: item.type for item in probe.get_all_topics_and_types()}
    topics = [topic for topic in ("/tf", "/tf_static") if topic in types]
    if not topics:
        raise RuntimeError("target frame conversion is needed, but the bag has no /tf or /tf_static")
    probe.set_filter(StorageFilter(topics=topics))
    classes = {topic: get_message(types[topic]) for topic in topics}
    archive = TfArchive()
    while probe.has_next():
        topic, data, _ = probe.read_next()
        archive.add_message(deserialize_message(data, classes[topic]), topic == "/tf_static")
    archive.finalize()
    return archive


POINTFIELD_DTYPES = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "i2",
    PointField.UINT16: "u2",
    PointField.INT32: "i4",
    PointField.UINT32: "u4",
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
}


def pointcloud_array(msg) -> np.ndarray:
    names, formats, offsets = [], [], []
    byte_order = ">" if msg.is_bigendian else "<"
    for field in msg.fields:
        if field.name in ("x", "y", "z", "rgb", "rgba") and field.count == 1:
            if field.datatype not in POINTFIELD_DTYPES:
                raise RuntimeError(f"unsupported datatype for field {field.name}: {field.datatype}")
            names.append(field.name)
            formats.append(byte_order + POINTFIELD_DTYPES[field.datatype])
            offsets.append(field.offset)
    required = {"x", "y", "z"}
    if not required.issubset(names) or not ({"rgb", "rgba"} & set(names)):
        raise RuntimeError(f"PointCloud2 needs x/y/z and rgb or rgba fields; found {names}")
    dtype = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step}
    )
    count = int(msg.width) * int(msg.height)
    if msg.height == 1 or msg.row_step == msg.width * msg.point_step:
        return np.frombuffer(msg.data, dtype=dtype, count=count)
    rows = [
        np.frombuffer(msg.data, dtype=dtype, count=msg.width, offset=row * msg.row_step)
        for row in range(msg.height)
    ]
    return np.concatenate(rows)


def extract_points(msg, transform: np.ndarray) -> np.ndarray:
    source = pointcloud_array(msg)
    xyz = np.column_stack((source["x"], source["y"], source["z"]))
    valid = np.isfinite(xyz).all(axis=1)
    xyz = xyz[valid].astype(np.float64, copy=False)
    if not np.allclose(transform, np.eye(4)):
        xyz = xyz @ transform[:3, :3].T + transform[:3, 3]

    color_name = "rgb" if "rgb" in source.dtype.names else "rgba"
    color = source[color_name][valid]
    if color.dtype.itemsize != 4:
        raise RuntimeError(f"{color_name} must be FLOAT32 or UINT32")
    color = color.view(color.dtype.byteorder + "u4").astype("<u4", copy=False)
    if color_name == "rgba":
        color = color & np.uint32(0x00FFFFFF)

    output = np.empty(len(xyz), dtype=PCD_POINT_DTYPE)
    output["x"], output["y"], output["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    output["rgb"] = color
    return output


def make_pcd_header(point_count: int) -> bytes:
    # Fixed-width decimal fields let us safely rewrite the header after streaming.
    return (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F U\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {point_count:020d}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {point_count:020d}\n"
        "DATA binary\n"
    ).encode("ascii")


class PcdWriter:
    def __init__(self, output_path: Path) -> None:
        self.output_path = output_path
        self.handle = output_path.open("wb+")
        self.handle.write(make_pcd_header(0))
        self.count = 0

    def write(self, points: np.ndarray) -> None:
        points.astype(PCD_POINT_DTYPE, copy=False).tofile(self.handle)
        self.count += len(points)

    def close(self) -> None:
        self.handle.seek(0)
        self.handle.write(make_pcd_header(self.count))
        self.handle.close()


def make_ply_header(point_count: int) -> bytes:
    # Fixed-width count permits an in-place update after streaming all vertices.
    return (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment generated by odin1_rosbag_pointcloud_exporter\n"
        f"element vertex {point_count:020d}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    ).encode("ascii")


class PlyWriter:
    def __init__(self, output_path: Path) -> None:
        self.output_path = output_path
        self.handle = output_path.open("wb+")
        self.handle.write(make_ply_header(0))
        self.count = 0

    def write(self, points: np.ndarray) -> None:
        output = np.empty(len(points), dtype=PLY_POINT_DTYPE)
        output["x"], output["y"], output["z"] = points["x"], points["y"], points["z"]
        rgb = points["rgb"]
        output["red"] = (rgb >> np.uint32(16)) & np.uint32(0xFF)
        output["green"] = (rgb >> np.uint32(8)) & np.uint32(0xFF)
        output["blue"] = rgb & np.uint32(0xFF)
        output.tofile(self.handle)
        self.count += len(points)

    def close(self) -> None:
        self.handle.seek(0)
        self.handle.write(make_ply_header(self.count))
        self.handle.close()


def create_writer(output_path: Path):
    suffix = output_path.suffix.lower()
    if suffix == ".pcd":
        return PcdWriter(output_path)
    if suffix == ".ply":
        return PlyWriter(output_path)
    raise RuntimeError("output extension must be .pcd or .ply")


def voxel_hash(ix: np.ndarray, iy: np.ndarray, iz: np.ndarray) -> np.ndarray:
    # Unsigned overflow is intentional and gives a deterministic spatial hash.
    with np.errstate(over="ignore"):
        return (
            ix.astype(np.uint64) * np.uint64(0x9E3779B185EBCA87)
            ^ iy.astype(np.uint64) * np.uint64(0xC2B2AE3D27D4EB4F)
            ^ iz.astype(np.uint64) * np.uint64(0x165667B19E3779F9)
        )


class DiskVoxelAccumulator:
    def __init__(self, temp_dir: Path, voxel_size: float, partitions: int) -> None:
        self.temp_dir = temp_dir
        self.voxel_size = voxel_size
        self.partitions = partitions
        self.paths = [temp_dir / f"voxel_{index:04d}.bin" for index in range(partitions)]
        self.handles: Dict[int, object] = {}
        self.pending: Dict[int, List[np.ndarray]] = collections.defaultdict(list)
        self.pending_frames = 0
        self.input_points = 0

    def _flush(self) -> None:
        for shard, chunks in self.pending.items():
            handle = self.handles.get(shard)
            if handle is None:
                handle = self.paths[shard].open("ab")
                self.handles[shard] = handle
            if len(chunks) == 1:
                chunks[0].tofile(handle)
            else:
                np.concatenate(chunks).tofile(handle)
        self.pending.clear()
        self.pending_frames = 0

    def add(self, points: np.ndarray) -> None:
        if not len(points):
            return
        self.input_points += len(points)
        xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(np.float64)
        keys = np.floor(xyz / self.voxel_size).astype(np.int64)

        # Remove duplicates inside this scan before writing temporary records.
        key_view = np.ascontiguousarray(keys).view(VOXEL_KEY_DTYPE).reshape(-1)
        _, unique_indices = np.unique(key_view, return_index=True)
        keys, points = keys[unique_indices], points[unique_indices]
        shard_ids = (voxel_hash(keys[:, 0], keys[:, 1], keys[:, 2]) % self.partitions).astype(np.int32)
        order = np.argsort(shard_ids, kind="stable")
        shard_ids, keys, points = shard_ids[order], keys[order], points[order]
        boundaries = np.flatnonzero(np.diff(shard_ids)) + 1
        for group in np.split(np.arange(len(points)), boundaries):
            shard = int(shard_ids[group[0]])
            records = np.empty(len(group), dtype=VOXEL_RECORD_DTYPE)
            records["ix"], records["iy"], records["iz"] = keys[group, 0], keys[group, 1], keys[group, 2]
            for name in ("x", "y", "z", "rgb"):
                records[name] = points[name][group]
            self.pending[shard].append(records)
        self.pending_frames += 1
        # Batch small writes. Fifty typical Odin scans use roughly 50--80 MiB.
        if self.pending_frames >= 50:
            self._flush()

    def finish(self, writer) -> int:
        self._flush()
        for handle in self.handles.values():
            handle.close()
        self.handles.clear()
        for index, path in enumerate(self.paths):
            if not path.exists() or path.stat().st_size == 0:
                continue
            records = np.fromfile(path, dtype=VOXEL_RECORD_DTYPE)
            keys = np.empty(len(records), dtype=VOXEL_KEY_DTYPE)
            keys["ix"], keys["iy"], keys["iz"] = records["ix"], records["iy"], records["iz"]
            _, unique_indices = np.unique(keys, return_index=True)
            output = np.empty(len(unique_indices), dtype=PCD_POINT_DTYPE)
            for name in ("x", "y", "z", "rgb"):
                output[name] = records[name][unique_indices]
            writer.write(output)
            del records, keys, output
            path.unlink()
            if (index + 1) % max(1, self.partitions // 10) == 0:
                print(f"  voxel reduction: {index + 1}/{self.partitions} partitions", flush=True)
        return writer.count


def progress_text(frames: int, input_points: int, started: float) -> str:
    elapsed = max(1e-6, time.monotonic() - started)
    return (
        f"  {frames} frames, {input_points:,} valid points, "
        f"{input_points / elapsed / 1e6:.2f} Mpoint/s"
    )


def export(args: argparse.Namespace) -> Tuple[int, int, str]:
    bag_path = args.bag.resolve()
    if not bag_path.exists():
        raise RuntimeError(f"bag path does not exist: {bag_path}")
    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists() and not args.force:
        raise RuntimeError(f"output already exists: {output_path} (use --force to overwrite)")

    reader, types = open_reader(bag_path, [args.topic])
    cloud_class = get_message(types[args.topic])
    tf_archive: Optional[TfArchive] = None
    discovered_source: Optional[str] = None
    first_cloud_stamp_ns: Optional[int] = None
    max_gap_ns = int(args.max_tf_gap * 1e9)
    writer = create_writer(output_path)
    frames = 0
    input_points = 0
    started = time.monotonic()
    temp_context = None
    accumulator = None

    try:
        if args.voxel_size > 0:
            temp_context = tempfile.TemporaryDirectory(prefix="odin1_pointcloud_", dir=args.temp_dir)
            accumulator = DiskVoxelAccumulator(
                Path(temp_context.name), args.voxel_size, args.partitions
            )

        message_index = 0
        while reader.has_next():
            _, data, _ = reader.read_next()
            msg = deserialize_message(data, cloud_class)
            current_stamp_ns = stamp_ns(msg.header.stamp)
            if first_cloud_stamp_ns is None:
                first_cloud_stamp_ns = current_stamp_ns
            elapsed_ns = current_stamp_ns - first_cloud_stamp_ns
            if elapsed_ns < int(args.start_offset * 1e9):
                continue
            if (
                args.end_offset is not None
                and elapsed_ns > int(args.end_offset * 1e9)
            ):
                print(
                    f"Reached end offset {args.end_offset:g}s after {frames} frames.",
                    flush=True,
                )
                break
            if message_index % args.every_nth_frame:
                message_index += 1
                continue
            message_index += 1
            source = normalize_frame(msg.header.frame_id)
            if not source:
                raise RuntimeError("PointCloud2 has an empty frame_id")
            if discovered_source is None:
                discovered_source = source
                print(f"Point cloud frame: {source}; target frame: {args.target_frame}", flush=True)
                if source != normalize_frame(args.target_frame):
                    print("Loading recorded TF data...", flush=True)
                    tf_archive = load_tf_archive(bag_path)
            if source == normalize_frame(args.target_frame):
                transform = np.eye(4, dtype=np.float64)
            else:
                assert tf_archive is not None
                transform = tf_archive.lookup(
                    args.target_frame, source, stamp_ns(msg.header.stamp), max_gap_ns
                )
            points = extract_points(msg, transform)
            input_points += len(points)
            frames += 1
            if accumulator is None:
                writer.write(points)
            else:
                accumulator.add(points)
            if frames % args.progress_every == 0:
                print(progress_text(frames, input_points, started), flush=True)

        if accumulator is not None:
            print("Reducing global voxel partitions...", flush=True)
            accumulator.finish(writer)
        writer.close()
        writer = None
    except Exception:
        if writer is not None:
            writer.handle.close()
        if output_path.exists():
            output_path.unlink()
        raise
    finally:
        if temp_context is not None:
            temp_context.cleanup()

    return frames, input_points, discovered_source or ""


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline accumulation of colored PointCloud2 data from a ROS 2 bag into binary PCD or PLY."
    )
    parser.add_argument("bag", type=Path, help="rosbag2 directory or MCAP/SQLite bag path")
    parser.add_argument("-o", "--output", type=Path, help="output .pcd or .ply path")
    parser.add_argument("--topic", default="/odin1/cloud_slam", help="PointCloud2 topic")
    parser.add_argument("--target-frame", default="odom", help="output coordinate frame")
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.05,
        help="global voxel size in metres (default: 0.05; use 0 to retain all points)",
    )
    parser.add_argument(
        "--partitions",
        type=int,
        default=256,
        help="temporary disk partitions for bounded-memory voxel reduction (default: 256)",
    )
    parser.add_argument("--temp-dir", type=Path, help="temporary data parent directory")
    parser.add_argument("--every-nth-frame", type=int, default=1, help="process every Nth cloud")
    parser.add_argument(
        "--start-offset",
        type=float,
        default=0.0,
        help="start this many seconds after the first cloud timestamp (default: 0)",
    )
    parser.add_argument(
        "--end-offset",
        type=float,
        help="stop after this many seconds from the first cloud timestamp",
    )
    parser.add_argument("--max-tf-gap", type=float, default=0.5, help="maximum TF extrapolation in seconds")
    parser.add_argument("--progress-every", type=int, default=100, help="progress interval in frames")
    parser.add_argument("--force", action="store_true", help="overwrite an existing output")
    args = parser.parse_args(argv)
    if args.output is None:
        bag_name = args.bag.name.rstrip("/")
        args.output = Path.cwd() / f"{bag_name}_odin1_colored_map.ply"
    if args.voxel_size < 0:
        parser.error("--voxel-size must be >= 0")
    if args.partitions < 1:
        parser.error("--partitions must be >= 1")
    if args.every_nth_frame < 1 or args.progress_every < 1:
        parser.error("frame intervals must be >= 1")
    if args.start_offset < 0:
        parser.error("--start-offset must be >= 0")
    if args.end_offset is not None and args.end_offset <= 0:
        parser.error("--end-offset must be > 0")
    if args.end_offset is not None and args.end_offset <= args.start_offset:
        parser.error("--end-offset must be greater than --start-offset")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    print(f"Bag: {args.bag.resolve()}")
    print(f"Output: {args.output.resolve()}")
    mode = "no voxel filtering" if args.voxel_size == 0 else f"global {args.voxel_size:g} m voxel filtering"
    print(f"Mode: {mode}", flush=True)
    try:
        frames, input_points, source = export(args)
    except KeyboardInterrupt:
        print("\nInterrupted; partial output and temporary files were removed.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    size_gib = args.output.stat().st_size / (1024 ** 3)
    print(
        f"Done: {frames} frames / {input_points:,} input points -> "
        f"{args.output} ({size_gib:.2f} GiB, frame={args.target_frame}, source={source})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
