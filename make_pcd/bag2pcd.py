#!/usr/bin/env python3
import argparse
import numpy as np

# ROS 2 imports
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message  # not used, but handy if you extend
from rclpy import ok as rclpy_ok  # rclpy init not required for deserialization
from rosidl_runtime_py.utilities import get_message

# PointCloud2 helpers
from sensor_msgs_py import point_cloud2 as pc2

# Open3D for writing .pcd
import open3d as o3d
from sensor_msgs.msg import PointField  # make sure this is imported

def _pf_datatype_to_numpy(dtype: int, endian_char: str) -> np.dtype:
    """Map sensor_msgs/PointField datatype -> numpy dtype with endianness."""
    m = {
        PointField.INT8:   "i1",
        PointField.UINT8:  "u1",
        PointField.INT16:  "i2",
        PointField.UINT16: "u2",
        PointField.INT32:  "i4",
        PointField.UINT32: "u4",
        PointField.FLOAT32:"f4",
        PointField.FLOAT64:"f8",
    }
    if dtype not in m:
        raise ValueError(f"Unsupported PointField datatype: {dtype}")
    return np.dtype(endian_char + m[dtype])

def pointcloud2_to_numpy(msg, fields=('x','y','z'), remove_nans=True):
    """
    Robust raw-buffer parser for PointCloud2 with mixed datatypes and arbitrary point_step.
    Returns (N, len(fields)) float32.
    """
    endian_char = '>' if msg.is_bigendian else '<'

    # Build structured dtype via dict form (so we can set explicit offsets & overall itemsize)
    names, formats, offsets = [], [], []
    available = set()
    for f in msg.fields:
        base_dt = _pf_datatype_to_numpy(f.datatype, endian_char)
        # Handle multi-count fields (e.g. float32[3]); most lidar fields have count==1
        fmt = (base_dt, (f.count,)) if getattr(f, "count", 1) and f.count > 1 else base_dt
        names.append(f.name)
        formats.append(fmt)
        offsets.append(f.offset)
        available.add(f.name)

    missing = [n for n in fields if n not in available]
    if missing:
        raise RuntimeError(f"Requested fields missing from cloud: {missing}. "
                           f"Available: {sorted(available)}")

    structured_dtype = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets, "itemsize": msg.point_step},
        align=False
    )

    n_points = msg.width * msg.height
    arr = np.frombuffer(msg.data, dtype=structured_dtype, count=n_points)

    # Stack requested scalar fields (x,y,z, intensity, etc.)
    cols = []
    for name in fields:
        col = arr[name]
        # If this field had count>1 (subarray), take the first component by default
        # (not typical for x/y/z/intensity, but keeps us safe)
        if isinstance(col.dtype, np.dtype) and col.dtype.subdtype is not None:
            col = col[..., 0]
        cols.append(col.astype(np.float32, copy=False))

    out = np.stack(cols, axis=-1)

    if remove_nans:
        mask = np.isfinite(out).all(axis=1)
        out = out[mask]

    return out



def read_first_pointcloud_from_bag(bag_path: str, topic: str):
    """
    Opens a ROS 2 bag and returns the first sensor_msgs/msg/PointCloud2 from `topic`.
    Returns (msg, timestamp_ns). Raises RuntimeError if not found.
    """
    storage_opts = StorageOptions(uri=bag_path, storage_id='sqlite3')  # most common
    # Converter options: adapt if your bag uses different serialization
    conv_opts = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )

    reader = SequentialReader()
    reader.open(storage_opts, conv_opts)

    # Query available topics & types in the bag so we can deserialize correctly
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}

    if topic not in topic_types:
        # Try to help: list similar topics
        available = "\n  - " + "\n  - ".join(topic_types.keys()) if topic_types else " (none)"
        raise RuntimeError(
            f"Topic '{topic}' not found in bag. Available topics:{available}"
        )

    # Prepare the type support to deserialize into a Python class
    msg_type_str = topic_types[topic]  # e.g. 'sensor_msgs/msg/PointCloud2'
    if msg_type_str != 'sensor_msgs/msg/PointCloud2':
        raise RuntimeError(
            f"Topic '{topic}' has type '{msg_type_str}', not PointCloud2."
        )
    MsgType = get_message(msg_type_str)

    # Iterate messages until we find one from the requested topic
    while reader.has_next():
        (topic_name, data, t) = reader.read_next()
        if topic_name == topic:
            msg = deserialize_message(data, MsgType)
            return msg, t  # t is timestamp in nanoseconds

    raise RuntimeError(f"No messages found on topic '{topic}'.")



def write_pcd_xyz(points_xyz: np.ndarray, out_path: str, ascii: bool = False):
    """
    Writes an (N,3) array as a .pcd file using Open3D.
    """
    if points_xyz.size == 0:
        raise RuntimeError("No points to write.")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_xyz.astype(np.float64))
    # Write .pcd
    success = o3d.io.write_point_cloud(out_path, pcd, write_ascii=ascii, compressed=not ascii)
    if not success:
        raise RuntimeError(f"Failed to write PCD to '{out_path}'.")


def main():
    parser = argparse.ArgumentParser(description="Extract one LiDAR PointCloud2 from a ROS 2 bag and save as .pcd")
    parser.add_argument("--bag", required=True, help="Path to the ROS 2 bag directory (the folder containing metadata.yaml).")
    parser.add_argument("--topic", required=True, help="PointCloud2 topic to extract (e.g. /points, /lidar/points)")
    parser.add_argument("--out", required=True, help="Output .pcd file path (e.g. out.pcd)")
    parser.add_argument("--ascii", action="store_true", help="Write ASCII PCD (default is compressed binary).")
    parser.add_argument("--with-intensity", action="store_true",
                        help="Also read 'intensity' field (saved only if you extend writing to include colors).")
    args = parser.parse_args()

    # 1) Read the first PointCloud2 from the bag
    msg, t = read_first_pointcloud_from_bag(args.bag, args.topic)

    # 2) Convert to NumPy
    fields = ('x', 'y', 'z', 'intensity') if args.with_intensity else ('x', 'y', 'z')
    pts = pointcloud2_to_numpy(msg, fields=fields, remove_nans=True)

    if pts.shape[1] == 3:
        xyz = pts[:, :3]
        write_pcd_xyz(xyz, args.out, ascii=args.ascii)
    else:
        # If you want to map intensity → grayscale for visualization, you could
        # extend this to set point colors. For now we just write XYZ.
        xyz = pts[:, :3]
        write_pcd_xyz(xyz, args.out, ascii=args.ascii)

    print(f"Wrote {xyz.shape[0]} points to {args.out}")


if __name__ == "__main__":
    main()
