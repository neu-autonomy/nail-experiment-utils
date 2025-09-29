#!/usr/bin/env python3
import argparse
import math
import os
import sys
import numpy as np
import open3d as o3d


def print_help():
    msg = r"""
Controls:
  Translate source:    W/S (±Z), A/D (±X), Q/E (±Y)
  Rotate source (deg): I/K (± about X), J/L (± about Y), U/O (± about Z)
  Step size:           [ decrease, ] increase
  ICP max distance:    ; decrease, ' increase
  Toggle ICP type:     P   (point-to-plane <-> point-to-point)
  Run ICP refine:      T
  Reset transform:     R
  Center view:         C
  Save transform:      M   (writes transform.npy)
"""
    print(msg)


def make_transform(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def rot_axis(axis, degrees):
    th = math.radians(degrees)
    c, s = math.cos(th), math.sin(th)
    if axis == "x":
        return np.array([[1, 0, 0],
                         [0, c, -s],
                         [0, s,  c]])
    if axis == "y":
        return np.array([[ c, 0, s],
                         [ 0, 1, 0],
                         [-s, 0, c]])
    if axis == "z":
        return np.array([[c, -s, 0],
                         [s,  c, 0],
                         [0,  0, 1]])
    raise ValueError("axis must be x/y/z")


def voxel_down_if_needed(pcd, voxel):
    if voxel is None or voxel <= 0:
        return pcd
    q = pcd.voxel_down_sample(voxel)
    if len(q.points) == 0:
        # Fallback if too aggressive
        return pcd
    return q


def align_pointclouds(
    src_path,
    tgt_path,
    voxel_vis=0.02,              # for fast visualization
    voxel_icp=0.02,              # for fast ICP preproc
    icp_max_dist=0.05,           # ICP correspondence distance
    normals_radius_factor=3.0,   # radius = factor * voxel_icp
):
    if not os.path.exists(src_path) or not os.path.exists(tgt_path):
        print(f"ERROR: file not found.\n  src: {src_path}\n  tgt: {tgt_path}")
        sys.exit(1)

    print_help()

    src_full = o3d.io.read_point_cloud(src_path)
    tgt_full = o3d.io.read_point_cloud(tgt_path)
    if src_full.is_empty() or tgt_full.is_empty():
        print("ERROR: One of the pointclouds is empty.")
        sys.exit(1)

    # Quick visual downsampling for interactivity
    src = voxel_down_if_needed(src_full, voxel_vis)
    tgt = voxel_down_if_needed(tgt_full, voxel_vis)

    # make copies of original points for reset
    src_start = np.asarray(src.points)
    tgt_start = np.asarray(tgt.points)

    # Colors for visualization
    tgt.paint_uniform_color([0.7, 0.7, 0.7])  # light gray (fixed)
    src.paint_uniform_color([0.2, 0.8, 0.9])  # cyan (moving)

    # State
    state = {
        "T": np.eye(4),                 # current transform applied to source
        "t_step": 0.02,                 # translation step (meters)
        "r_step": 2.0,                  # rotation step (degrees)
        "icp_dist": icp_max_dist,
        "icp_point_to_plane": True,     # toggle ICP type
        "voxel_icp": voxel_icp,
        "normals_radius_factor": normals_radius_factor,
        "src_draw": src,
        "tgt_draw": tgt,
        "src_full": src_full,
        "tgt_full": tgt_full,
    }

    # Precompute normals for target (for point-to-plane) on a downsampled version
    def compute_icp_clouds():
        vs = state["voxel_icp"]
        src_icp = voxel_down_if_needed(state["src_full"].transform(state["T"].copy()), vs)
        tgt_icp = voxel_down_if_needed(state["tgt_full"], vs)
        rad = max(1e-6, state["normals_radius_factor"] * vs)
        if state["icp_point_to_plane"]:
            tgt_icp.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=rad, max_nn=30)
            )
        return src_icp, tgt_icp

    def update_title(vis):
        title = f"[Open3D] Align: t_step={state['t_step']:.3f}m  r_step={state['r_step']:.1f}°  ICPdist={state['icp_dist']:.3f}  ICP={'Pt2Plane' if state['icp_point_to_plane'] else 'Pt2Pt'}"
        vis.get_render_option().point_size = 2.0
        try:
            vis.get_view_control().convert_to_pinhole_camera_parameters()  # no-op to ensure window exists
            vis.create_window(window_name=title, visible=False)  # won't actually create again
        except:
            pass  # harmless
        return

    def apply_delta_transform(vis, dR=None, dt=None):
        T = state["T"]
        R = T[:3, :3]
        t = T[:3, 3]

        if dR is not None:
            R = dR @ R
        if dt is not None:
            t = t + dt

        T_new = make_transform(R, t)
        state["T"] = T_new

        # Update the drawable source (work on a copy of the downsampled vis cloud)
        state["src_draw"].points = o3d.utility.Vector3dVector(
            np.asarray(src.points) @ T_new[:3, :3].T + T_new[:3, 3]
        )
        # Keep color
        vis.update_geometry(state["src_draw"])
        update_title(vis)
        vis.update_renderer()

    def run_icp(vis):
        print("Running ICP...")
        src_icp, tgt_icp = compute_icp_clouds()

        if state["icp_point_to_plane"]:
            est = o3d.pipelines.registration.TransformationEstimationPointToPlane()
        else:
            est = o3d.pipelines.registration.TransformationEstimationPointToPoint()

        result = o3d.pipelines.registration.registration_icp(
            src_icp, tgt_icp, state["icp_dist"], np.eye(4), est,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50)
        )
        print(f"ICP fitness={result.fitness:.4f}, inlier_rmse={result.inlier_rmse:.4f}")

        # Apply the ICP delta on top of current transform
        state["T"] = result.transformation @ state["T"]

        # Update vis geometry positions
        T_new = state["T"]
        state["src_draw"].points = o3d.utility.Vector3dVector(
            np.asarray(src.points) @ T_new[:3, :3].T + T_new[:3, 3]
        )
        vis.update_geometry(state["src_draw"])
        update_title(vis)
        vis.update_renderer()

    def reset_transform(vis):
        print("Reset transform.")
        state["T"] = np.eye(4)
        state["src_draw"].points = o3d.utility.Vector3dVector(src_start)
        vis.update_geometry(state["src_draw"])
        update_title(vis)
        vis.update_renderer()

    def save_transform(_vis):
        np.save("transform.npy", state["T"])
        print("Saved 4x4 transform to transform.npy\n", state["T"])

    def center_view(vis):
        vis.reset_view_point(True)

    # ---- Visualizer & callbacks ----
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Open3D PointCloud Align", width=1280, height=800)
    vis.add_geometry(state["tgt_draw"])
    vis.add_geometry(state["src_draw"])
    update_title(vis)

    # Key helpers
    def dtrans(dx=0, dy=0, dz=0):
        def _cb(vis):
            apply_delta_transform(vis, dt=np.array([dx, dy, dz]))
            return False
        return _cb

    def drot(axis, deg):
        def _cb(vis):
            apply_delta_transform(vis, dR=rot_axis(axis, deg))
            return False
        return _cb

    def step_inc(vis, sign):
        if sign > 0:
            state["t_step"] *= 1.25
            state["r_step"] *= 1.25
        else:
            state["t_step"] /= 1.25
            state["r_step"] /= 1.25
        state["t_step"] = max(1e-4, state["t_step"])
        state["r_step"] = max(0.1, state["r_step"])
        update_title(vis)
        vis.update_renderer()
        return False

    def icp_dist_inc(vis, sign):
        if sign > 0:
            state["icp_dist"] *= 1.25
        else:
            state["icp_dist"] /= 1.25
        state["icp_dist"] = max(1e-4, state["icp_dist"])
        print(f"ICP max distance: {state['icp_dist']:.4f}")
        update_title(vis)
        vis.update_renderer()
        return False

    def toggle_icp_type(vis):
        state["icp_point_to_plane"] = not state["icp_point_to_plane"]
        print(f"ICP type: {'Point-to-Plane' if state['icp_point_to_plane'] else 'Point-to-Point'}")
        update_title(vis)
        vis.update_renderer()
        return False

    # Bind keys
    # Movement (uses current t_step)
    def bind_move(key, dx=0, dy=0, dz=0):
        vis.register_key_callback(ord(key), lambda v: dtrans(
            dx*state["t_step"], dy*state["t_step"], dz*state["t_step"])(v))

    # Rotation (uses current r_step)
    def bind_rot(key, axis, sign):
        vis.register_key_callback(ord(key), lambda v: drot(axis, sign*state["r_step"])(v))

    bind_move('A', dx=-1); bind_move('D', dx=+1)
    bind_move('Q', dy=+1);  bind_move('E', dy=-1)
    bind_move('W', dz=+1);  bind_move('S', dz=-1)

    bind_rot('I', 'x', +1); bind_rot('K', 'x', -1)
    bind_rot('J', 'y', +1); bind_rot('L', 'y', -1)
    bind_rot('U', 'z', +1); bind_rot('O', 'z', -1)

    vis.register_key_callback(ord('['), lambda v: step_inc(v, -1))
    vis.register_key_callback(ord(']'), lambda v: step_inc(v, +1))
    vis.register_key_callback(ord(';'), lambda v: icp_dist_inc(v, -1))
    vis.register_key_callback(ord("'"), lambda v: icp_dist_inc(v, +1))

    vis.register_key_callback(ord('P'), toggle_icp_type)
    vis.register_key_callback(ord('T'), run_icp)
    vis.register_key_callback(ord('R'), reset_transform)
    vis.register_key_callback(ord('M'), save_transform)
    vis.register_key_callback(ord('C'), center_view)

    vis.run()
    vis.destroy_window()

    return state["T"]


def main():
    parser = argparse.ArgumentParser(description="Minimal interactive point cloud alignment with ICP (Open3D).")
    parser.add_argument("--src", required=True, help="Path to source .pcd (moving)")
    parser.add_argument("--tgt", required=True, help="Path to target .pcd (fixed)")
    parser.add_argument("--voxel_vis", type=float, default=0.02, help="Visualization voxel size (downsample)")
    parser.add_argument("--voxel_icp", type=float, default=0.02, help="ICP voxel size (downsample)")
    parser.add_argument("--icp_max_dist", type=float, default=0.05, help="ICP max correspondence distance")
    args = parser.parse_args()

    T = align_pointclouds(
        args.src, args.tgt,
        voxel_vis=args.voxel_vis,
        voxel_icp=args.voxel_icp,
        icp_max_dist=args.icp_max_dist,
    )
    # Also print the final transform for convenience
    np.set_printoptions(precision=5, suppress=True)
    print("\nFinal transform (source->target):\n", T)


if __name__ == "__main__":
    main()
