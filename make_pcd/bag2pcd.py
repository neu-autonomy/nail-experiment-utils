#!/usr/bin/env python3
"""bag2pcd.py

Extract the first sensor_msgs/msg/PointCloud2 message from a ROS 2 bag
and write it out as a .pcd file (Open3D). This version includes runtime
sanity checks, clearer error messages, and more documentation.

Typical usage (from this repo):
  python3 bag2pcd.py --bag /path/to/rosbag_dir --topic /ouster/points --out map.pcd

Notes / checks performed:
- Verifies the bag path looks like a rosbag2 sqlite3 directory (metadata.yaml present).
- Verifies output directory exists or is creatable.
- Provides actionable error messages when dependencies are missing.
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import logging
from typing import Tuple

import numpy as np

# We'll import (and later verify) ROS 2 and Open3D-related modules at runtime.
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    # sensor_msgs helpers & types used in parsing
    from sensor_msgs.msg import PointField
except Exception:
    # Don't fail immediately; we'll provide a nicer message in verify_runtime()
    SequentialReader = None  # type: ignore
    StorageOptions = None  # type: ignore
    ConverterOptions = None  # type: ignore
    deserialize_message = None  # type: ignore
    get_message = None  # type: ignore
    PointField = None  # type: ignore

try:
    # optional helper module (not used directly here but commonly available)
    from sensor_msgs_py import point_cloud2 as pc2  # noqa: F401
except Exception:
    pc2 = None  # type: ignore

try:
    import open3d as o3d
except Exception:
    o3d = None  # type: ignore


def verify_runtime() -> None:
    """Verify that expected runtime dependencies are available and give helpful errors.

    Exits the program with a non-zero status and human-friendly message when
    critical dependencies are missing.
    """
    missing = []
    # Check ROS 2 python API
    if importlib.util.find_spec("rosbag2_py") is None or SequentialReader is None:
        missing.append("rosbag2_py (ROS 2 Python API)")
    if importlib.util.find_spec("rclpy") is None or deserialize_message is None:
        missing.append("rclpy")
    if importlib.util.find_spec("rosidl_runtime_py") is None or get_message is None:
        missing.append("rosidl_runtime_py")
    if importlib.util.find_spec("sensor_msgs") is None or PointField is None:
        missing.append("sensor_msgs (ROS messages)")

    # Check Open3D
    if importlib.util.find_spec("open3d") is None or o3d is None:
        missing.append("open3d (for writing .pcd files)")

    if missing:
        logging.error("Missing required Python packages: %s", ", ".join(missing))
        logging.error(
            "Install ROS 2 Python packages (source a ROS 2 workspace) and 'pip install open3d' as needed."
        )
        sys.exit(2)


def _pf_datatype_to_numpy(dtype: int, endian_char: str) -> np.dtype:
    """Map sensor_msgs/PointField datatype -> numpy dtype with endianness.

    Raises ValueError for unsupported datatypes.
    """
    m = {
        PointField.INT8: "i1",
        PointField.UINT8: "u1",
        PointField.INT16: "i2",
        PointField.UINT16: "u2",
        PointField.INT32: "i4",
        PointField.UINT32: "u4",
        PointField.FLOAT32: "f4",
        PointField.FLOAT64: "f8",
    }
    if dtype not in m:
        raise ValueError(f"Unsupported PointField datatype: {dtype}")
    return np.dtype(endian_char + m[dtype])


def pointcloud2_to_numpy(msg, fields=("x", "y", "z"), remove_nans=True) -> np.ndarray:
    """Parse a sensor_msgs/PointCloud2 raw buffer into a (N, len(fields)) float32 array.

    This function is robust to mixed datatypes and arbitrary point_step/offsets.
    """
    endian_char = ">" if msg.is_bigendian else "<"

    # Build structured dtype with explicit offsets so we can use frombuffer
    names, formats, offsets = [], [], []
    available = set()
    for f in msg.fields:
        base_dt = _pf_datatype_to_numpy(f.datatype, endian_char)
        fmt = (base_dt, (f.count,)) if getattr(f, "count", 1) and f.count > 1 else base_dt
        names.append(f.name)
        formats.append(fmt)
        offsets.append(f.offset)
        available.add(f.name)

    missing = [n for n in fields if n not in available]
    if missing:
        raise RuntimeError(
            f"Requested fields missing from cloud: {missing}. Available: {sorted(available)}"
        )

    structured_dtype = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step},
        align=False,
    )

    n_points = int(msg.width) * int(msg.height)
    arr = np.frombuffer(msg.data, dtype=structured_dtype, count=n_points)

    # Stack requested scalar fields (x,y,z, intensity, etc.)
    cols = []
    for name in fields:
        col = arr[name]
        # If this field had count>1 (subarray), take the first component by default
        if col.dtype.subdtype is not None:
            col = col[..., 0]
        cols.append(col.astype(np.float32, copy=False))

    out = np.stack(cols, axis=-1)

    if remove_nans:
        mask = np.isfinite(out).all(axis=1)
        out = out[mask]

    return out


def read_first_pointcloud_from_bag(bag_path: str, topic: str):
    """Open a ROS 2 bag and return the first PointCloud2 message on `topic`.

    Returns a tuple (msg, timestamp_ns). Raises RuntimeError on missing topic/messages.
    """
    # Basic bag path checks: bag_path should be a directory containing metadata.yaml
    if not os.path.isdir(bag_path):
        raise RuntimeError(f"Bag path is not a directory: {bag_path}")
    if not os.path.exists(os.path.join(bag_path, "metadata.yaml")):
        raise RuntimeError(f"Bag directory does not contain metadata.yaml: {bag_path}")

    storage_opts = StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv_opts = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")

    reader = SequentialReader()
    reader.open(storage_opts, conv_opts)

    # Query available topics & types in the bag so we can deserialize correctly
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if topic not in topic_types:
        available = "\n  - " + "\n  - ".join(sorted(topic_types.keys())) if topic_types else " (none)"
        raise RuntimeError(f"Topic '{topic}' not found in bag. Available topics:{available}")

    msg_type_str = topic_types[topic]
    if msg_type_str != "sensor_msgs/msg/PointCloud2":
        raise RuntimeError(f"Topic '{topic}' has type '{msg_type_str}', not PointCloud2.")

    MsgType = get_message(msg_type_str)

    # Iterate messages until we find one from the requested topic
    while reader.has_next():
        (topic_name, data, t) = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, MsgType)
            return msg, t

    raise RuntimeError(f"No messages found on topic '{topic}'.")


def write_pcd_xyz(points_xyz: np.ndarray, out_path: str, ascii: bool = False) -> None:
    """Write an (N,3) numpy array to a PCD file using Open3D.

    Raises RuntimeError on failure.
    """
    if points_xyz.size == 0:
        raise RuntimeError("No points to write.")

    # Ensure parent directory exists
    parent = os.path.dirname(out_path) or "."
    os.makedirs(parent, exist_ok=True)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz.astype(np.float64))
    success = o3d.io.write_point_cloud(out_path, pcd, write_ascii=ascii, compressed=not ascii)
    if not success:
        raise RuntimeError(f"Failed to write PCD to '{out_path}'.")


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract one LiDAR PointCloud2 from a ROS 2 bag and save as .pcd"
    )
    parser.add_argument("--bag", required=True, help="Path to the ROS 2 bag directory (contains metadata.yaml).")
    parser.add_argument("--topic", required=True, help="PointCloud2 topic to extract (e.g. /points)")
    parser.add_argument("--out", required=True, help="Output .pcd file path (e.g. out.pcd)")
    parser.add_argument("--ascii", action="store_true", help="Write ASCII PCD (default is compressed binary).")
    parser.add_argument(
        "--with-intensity",
        action="store_true",
        help="Also read 'intensity' field (saved only if you extend writing to include colors).",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")
    args = parse_args(argv)

    # Sanity checks for runtime environment
    verify_runtime()

    # Read the first PointCloud2 from the bag
    msg, t = read_first_pointcloud_from_bag(args.bag, args.topic)

    # Convert to NumPy
    fields = ("x", "y", "z", "intensity") if args.with_intensity else ("x", "y", "z")
    pts = pointcloud2_to_numpy(msg, fields=fields, remove_nans=True)

    if pts.shape[1] < 3:
        raise RuntimeError(f"Unexpected point shape: {pts.shape}; need at least x,y,z.")

    xyz = pts[:, :3]
    write_pcd_xyz(xyz, args.out, ascii=args.ascii)

    logging.info("Wrote %d points to %s", xyz.shape[0], args.out)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        logging.exception("Error: %s", e)
        # Non-zero exit code for errors
        sys.exit(1)
