make_pcd — README

This directory contains small utilities to record a short ROS 2 bag of a LiDAR/pointcloud topic and convert the first PointCloud2 message into a .pcd file suitable for inspection or transfer.

Files
- `pcd_maker.sh` — top-level helper script that:
  1. records a short ros2 bag of a configured topic,
  2. runs `bag2pcd.py` to extract the first PointCloud2 and write a .pcd file,
  3. copies the resulting .pcd to a remote host via `scp`.
  The script contains runtime checks, a cleanup trap, and helpful exit codes.

- `bag2pcd.py` — Python utility that:
  - opens a ROS 2 rosbag2 directory (sqlite3 style),
  - finds the first message on the requested `PointCloud2` topic,
  - converts the raw PointCloud2 buffer to a NumPy array and writes an XYZ .pcd using Open3D.
  It includes runtime dependency checks, input validation, and clear error messages.

Prerequisites
- A ROS 2 installation (matching the rosbag versions used to record). Make sure you source the ROS 2 environment in the shell where you run the tools. Example for Humble:

```zsh
source /opt/ros/humble/setup.zsh
# or if you use a workspace overlay
# source ~/ros2_ws/install/setup.zsh
```

- Python 3.8+ (your ROS 2 distro's python is recommended). Install Open3D in the same Python environment you will run `bag2pcd.py` with:

```zsh
pip install --user open3d
```

- System commands required for `pcd_maker.sh`: `ros2`, `python3`, `scp`.

Quick examples

1) Record a short bag, convert to PCD, and upload to a remote host:

```zsh
./pcd_maker.sh user@host:/path/to/save/
```

This will:
- record the topic configured inside `pcd_maker.sh` (`TOPIC`, default `/ouster/points`) for the configured duration (`DUR`, default 5s),
- run `bag2pcd.py --bag <bag_dir> --topic <topic> --out <bag_dir>/map.pcd`,
- `scp` the resulting `map.pcd` to the `DEST` you provided.

2) Run the converter manually (useful for debugging or when you already have a bag):

```zsh
python3 make_pcd/bag2pcd.py --bag /path/to/rosbag_dir --topic /ouster/points --out /tmp/map.pcd
```

Notes and tips
- `pcd_maker.sh` contains variables you can change at the top of the file if you want a different `TOPIC`, `DUR`, or `OUT_DIR`.
- If you get import resolution errors in your editor for ROS packages (e.g., `rosbag2_py` or `rclpy`), it usually means you haven't sourced your ROS 2 environment inside the editor's Python/terminal environment. Sourcing the correct setup file or launching your editor from a sourced shell will fix those warnings.
- `bag2pcd.py` checks that the bag directory contains `metadata.yaml` (the usual rosbag2 sqlite layout). If your bag uses a different storage backend, the `StorageOptions` in the script may need to be adjusted.

Exit codes (from `pcd_maker.sh`)
- 1 — usage / bad arguments
- 2 — missing required command (ros2/python3/scp)
- 3 — converter script missing / unreadable
- 4 — cannot write to local OUT_DIR
- 5 — empty TOPIC variable
- 6 — invalid DUR value
- 7 — converter returned non-zero (conversion failed)
- 8 — expected PCD not found after conversion
- 9 — PCD file was empty
- 10 — scp failed (upload failed)

`bag2pcd.py` exit behavior
- Exits with code 0 on success.
- If runtime Python dependencies are missing (ROS 2 Python packages or Open3D) it prints an actionable message and exits with code 2.
- On other runtime exceptions it exits with code 1 and logs a stack trace.

Troubleshooting checklist
- "ros2 bag record" doesn't start or exits immediately: ensure ROS 2 environment is sourced and your network/interface for the LiDAR is configured correctly.
- Conversion fails with "Topic not found" or wrong message type: list topics in the bag to confirm the exact topic name and type:

```zsh
# Quick topic/type listing using rosbag2_py isn't always convenient; use the provided Python utility to print available topics:
python3 -c 'from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions; r=SequentialReader(); r.open(StorageOptions(uri="/path/to/bag", storage_id="sqlite3"), ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")); print([t.name+" -> "+t.type for t in r.get_all_topics_and_types()])'
```

- Editor shows import errors but the script runs fine from a sourced shell — this is normal if the editor's Python environment is different. Launch the editor from a terminal that has the ROS environment sourced, or configure the editor to use that interpreter.

Customization ideas
- Add CLI flags to `pcd_maker.sh` to override `TOPIC`, `DUR`, and `OUT_DIR` from the command line.
- Extend `bag2pcd.py` to write intensity as point colors (the script already reads intensity if `--with-intensity` is passed; you can map it to greyscale color vectors before writing).
- Add a `--no-upload` flag to `pcd_maker.sh` for local-only operation.

If you'd like, I can:
- add command-line flag parsing to `pcd_maker.sh` (getopts) and support `--topic`, `--dur`, `--out-dir`, `--no-upload`;
- add a small test that generates a synthetic PointCloud2-like buffer and verifies `pointcloud2_to_numpy` output.

License / attribution
- This README and the scripts are project utility code; please follow your project's licensing.

---
Generated on: 2025-10-16
