#!/usr/bin/env python3
"""
This script provides an interactive 3D alignment tool for two point clouds.
It uses the Open3D library to visualize the point clouds and allows for manual
(keyboard-based) transformation alignment, as well as running the Iterative
Closest Point (ICP) algorithm to refine the alignment.
"""
import argparse
import math
import os
import sys
import numpy as np
import open3d as o3d
from typing import Tuple, Dict, Any, Iterable


def print_help():
    """Prints the control keys and their functions to the console."""
    msg = r"""
Controls:
  Translate: W/S (±Z), A/D (±X), Q/E (±Y)
  Rotate:    I/K (±X), J/L (±Y), U/O (±Z)
  Step size: [ down, ] up
  ICP dist:  ; down, ' up
  ICP type:  P (toggle)
  Run ICP:   T
  Reset:     R
  Center:    C
  Save:      M
  Help:      H
"""
    print(msg)


def make_transform(R=None, t=None):
    """Creates a 4x4 homogeneous transformation matrix.

    Args:
        R (np.ndarray, optional): A 3x3 rotation matrix. Defaults to None.
        t (np.ndarray, optional): A 3x1 translation vector. Defaults to None.

    Returns:
        np.ndarray: A 4x4 transformation matrix.
    """
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def rot_axis(axis, degrees):
    """Creates a 3x3 rotation matrix for a given axis and angle.

    Args:
        axis (str): The axis of rotation ('x', 'y', or 'z').
        degrees (float): The rotation angle in degrees.

    Returns:
        np.ndarray: A 3x3 rotation matrix.
    """
    th = math.radians(degrees)
    c, s = math.cos(th), math.sin(th)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    if axis == "z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    raise ValueError("axis must be x/y/z")


def voxel_down_if_needed(pcd, voxel):
    """Downsamples a point cloud using a voxel grid if the voxel size is valid.

    Args:
        pcd (o3d.geometry.PointCloud): The input point cloud.
        voxel (float): The voxel size for downsampling. If None or <= 0, no
            downsampling is performed.

    Returns:
        o3d.geometry.PointCloud: The downsampled point cloud, or the original
            if no downsampling was performed.
    """
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
    voxel_vis=0.02,  # for fast visualization
    voxel_icp=0.02,  # for fast ICP preproc
    icp_max_dist=0.05,  # ICP correspondence distance
    normals_radius_factor=3.0,  # radius = factor * voxel_icp
):
    """Interactively aligns a source point cloud to a target point cloud.

    This function opens a window displaying the two point clouds. The user can
    manually transform the source point cloud using keyboard controls and can
    run the ICP algorithm to refine the alignment.

    Args:
        src_path (str): Path to the source point cloud file.
        tgt_path (str): Path to the target point cloud file.
        voxel_vis (float, optional): Voxel size for downsampling for visualization.
            Defaults to 0.02.
        voxel_icp (float, optional): Voxel size for downsampling before ICP.
            Defaults to 0.02.
        icp_max_dist (float, optional): Maximum correspondence distance for ICP.
            Defaults to 0.05.
        normals_radius_factor (float, optional): Factor to determine the radius
            for normal estimation, relative to `voxel_icp`. Defaults to 3.0.

    Returns:
        np.ndarray: The final 4x4 transformation matrix that aligns the source
            point cloud to the target.
    """
    if not os.path.exists(src_path) or not os.path.exists(tgt_path):
        print(f"ERROR: file not found.\n  src: {src_path}\n  tgt: {tgt_path}")
        sys.exit(1)

    print("Press 'H' in the viewer to see the keybindings.")

    src_full = o3d.io.read_point_cloud(src_path)
    tgt_full = o3d.io.read_point_cloud(tgt_path)
    if src_full.is_empty() or tgt_full.is_empty():
        print("ERROR: One of the pointclouds is empty.")
        sys.exit(1)

    # Quick visual downsampling for interactivity
    src = voxel_down_if_needed(src_full, voxel_vis)
    tgt = voxel_down_if_needed(tgt_full, voxel_vis)

    src_points_original = np.asarray(src.points).copy()

    # Colors for visualization
    tgt.paint_uniform_color([0.7, 0.7, 0.7])  # light gray (fixed)
    src.paint_uniform_color([0.2, 0.8, 0.9])  # cyan (moving)

    # State dictionary to hold mutable variables
    state = {
        "T": np.eye(4),  # current transform applied to source
        "t_step": 0.02,  # translation step (meters)
        "r_step": 2.0,  # rotation step (degrees)
        "icp_dist": icp_max_dist,
        "icp_point_to_plane": True,  # toggle ICP type
        "voxel_icp": voxel_icp,
        "normals_radius_factor": normals_radius_factor,
        "src_draw": src,
        "src_points_original": src_points_original,
        "tgt_draw": tgt,
        "src_full": src_full,
        "tgt_full": tgt_full,
    }

    # def compute_icp_clouds():
    #     """Prepares the point clouds for ICP by downsampling and computing normals."""
    #     vs = state["voxel_icp"]

    #     # Make copies before transforming/downsampling
    #     src_full_copy = o3d.geometry.PointCloud(state["src_full"])
    #     src_full_copy.transform(state["T"])  # safe: operates on the copy
    #     src_icp = voxel_down_if_needed(src_full_copy, vs)

    #     tgt_icp = o3d.geometry.PointCloud(state["tgt_full"])
    #     tgt_icp = voxel_down_if_needed(tgt_icp, vs)

    #     if state["icp_point_to_plane"]:
    #         rad = max(1e-6, state["normals_radius_factor"] * vs)
    #         tgt_icp.estimate_normals(
    #             o3d.geometry.KDTreeSearchParamHybrid(radius=rad, max_nn=30)
    #         )

    #     return src_icp, tgt_icp

    def compute_icp_clouds(voxel):
        """Downsample + (re)compute normals for the current T and given voxel."""
        # Transform a fresh copy of the full source by current state["T"]
        src_full_copy = o3d.geometry.PointCloud(state["src_full"])
        src_full_copy.transform(state["T"])
        src_icp = voxel_down_if_needed(src_full_copy, voxel)

        tgt_icp = voxel_down_if_needed(state["tgt_full"], voxel)

        if state["icp_point_to_plane"]:
            rad = max(1e-6, state["normals_radius_factor"] * voxel)
            # For point-to-plane, target NEEDS normals; source normals are optional.
            tgt_icp.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=rad, max_nn=50)
            )
            # This helps avoid inconsistent normal flips on thin structures
            try:
                tgt_icp.orient_normals_consistent_tangent_plane(100)
            except Exception:
                pass
        return src_icp, tgt_icp

    def update_title(vis):
        """Updates the visualizer window title with the current state."""
        vis.get_render_option().point_size = 2.0
        try:
            vis.get_view_control().convert_to_pinhole_camera_parameters()  # no-op to ensure window exists
            vis.create_window(
                window_name="Open3D PointCloud Align", visible=False
            )  # won't actually create again
        except:
            pass  # harmless
        return

    def apply_delta_transform(vis, dR=None, dt=None):
        """Applies a small rotation and/or translation to the source point cloud."""
        T = state["T"]
        R = T[:3, :3]
        t = T[:3, 3]

        # apply the rotation in the world frame
        if dR is not None:
            R = dR @ R

        # apply the translation in the world frame
        if dt is not None:
            t = t + dt

        T_new = make_transform(R, t)
        state["T"] = T_new

        # Update the drawable source (work on a copy of the downsampled vis cloud)
        state["src_draw"].points = o3d.utility.Vector3dVector(
            (state["src_points_original"] @ T_new[:3, :3].T + T_new[:3, 3]).copy()
        )

        # Keep color
        vis.update_geometry(state["src_draw"])
        vis.update_renderer()

    def run_icp(vis):
        """Coarse->fine ICP with robust loss and distances tied to voxel size."""

        align_result = perform_iterative_alignment(
            state["src_full"],
            state["tgt_full"],
            init_T=state["T"],
        )
        T = align_result["T"]
        print(f"ICP fitness={align_result['fitness']:.4f}, inlier_rmse={align_result['inlier_rmse']:.4f}")


        # Commit and redraw
        state["T"] = T
        state["src_draw"].points = o3d.utility.Vector3dVector(
            (state["src_points_original"] @ T[:3, :3].T + T[:3, 3]).copy()
        )
        vis.update_geometry(state["src_draw"])
        update_title(vis)
        vis.update_renderer()

        print("ICP done.")

        # print the rotation and translation separately, to 2 decimal places
        rot, trans = T[:3, :3], T[:3, 3]
        print(f"Estimated rotation:\n{rot.round(2)}")
        print(f"Estimated translation:\n{trans.round(2)}")
        return False


    def perform_iterative_alignment(
        src: o3d.geometry.PointCloud,
        tgt: o3d.geometry.PointCloud,
        init_T: np.ndarray,
        *,
        base_voxel: float = 0.5,
        scales: Iterable[float] = (4.0, 2.0, 1.0),
        iters: Iterable[int] = (40, 25, 10),
        max_nn: int = 50,
        corr_factor: float = 2.5,
        tukey_k: float = 4.685
    ) -> Dict[str, Any]:
        """
        Multi-scale point-to-plane ICP with a robust loss.

        Args:
            src, tgt: Open3D point clouds (open3d.cpu.pybind.geometry.PointCloud).
            init_T: (4,4) numpy array giving a rough initial transform from src->tgt.
            base_voxel: 'finest' voxel size in meters (outdoor defaults: 0.5m).
            scales: multipliers for coarse->fine pyramid (e.g., 4x, 2x, 1x).
            iters: ICP iterations per scale (same length as 'scales').
            max_nn: neighbors for normal estimation at each scale.
            corr_factor: correspondence max distance = corr_factor * voxel_size_at_scale.
            tukey_k: Tukey loss parameter (robust to outliers).

        Returns:
            {
            "T": (4,4) refined transform src->tgt,
            "fitness": float,
            "inlier_rmse": float,
            "info_per_scale": [registration::RegistrationResult, ...]
            }
        """
        assert init_T.shape == (4, 4), "init_T must be 4x4"
        assert len(tuple(scales)) == len(tuple(iters)), "scales and iters length mismatch"

        # Work on copies to avoid modifying inputs
        src_in = o3d.geometry.PointCloud(src)
        tgt_in = o3d.geometry.PointCloud(tgt)

        T = init_T.copy()
        info_per_scale = []

        # Robust point-to-plane estimator
        loss = o3d.pipelines.registration.TukeyLoss(k=tukey_k)
        estimator = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)

        for s, n_iter in zip(scales, iters):
            vox = base_voxel * float(s)

            # Downsample
            src_ds = src_in.voxel_down_sample(vox)
            tgt_ds = tgt_in.voxel_down_sample(vox)

            # Estimate normals (radius ~ 2.5 * voxel works well outdoors)
            nradius = 2.5 * vox
            src_ds.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=nradius, max_nn=max_nn)
            )
            tgt_ds.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=nradius, max_nn=max_nn)
            )
            # Orients normals consistently for ICP stability
            src_ds.orient_normals_consistent_tangent_plane(10)
            tgt_ds.orient_normals_consistent_tangent_plane(10)

            # Set correspondence distance (a bit larger than voxel)
            max_corr = corr_factor * vox

            # Run ICP at this scale
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=int(n_iter)
            )
            result = o3d.pipelines.registration.registration_icp(
                src_ds, tgt_ds, max_corr, T, estimator, criteria
            )
            T = result.transformation
            info_per_scale.append(result)

        # Final pass metrics on finest resolution (optional)
        finest_vox = base_voxel * float(scales[-1])
        max_corr_final = corr_factor * finest_vox
        src_fin = src_in.voxel_down_sample(finest_vox)
        tgt_fin = tgt_in.voxel_down_sample(finest_vox)
        src_fin.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=2.5*finest_vox, max_nn=max_nn))
        tgt_fin.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=2.5*finest_vox, max_nn=max_nn))
        final_res = o3d.pipelines.registration.evaluate_registration(
            src_fin, tgt_fin, max_corr_final, T
        )

        return {
            "T": T,
            "fitness": float(final_res.fitness),
            "inlier_rmse": float(final_res.inlier_rmse),
            "info_per_scale": info_per_scale,
        }


    def reset_transform(vis):
        """Resets the source point cloud to its original position."""
        print("Reset transform.")
        state["T"] = np.eye(4)
        # restore the *original* downsampled source points (use a fresh numpy copy)
        state["src_draw"].points = o3d.utility.Vector3dVector(
            state["src_points_original"].copy()
        )
        vis.update_geometry(state["src_draw"])
        update_title(vis)
        vis.reset_view_point(True)  # optional: recenters view
        vis.update_renderer()

    def save_transform(_vis):
        """Saves the current transformation matrix to a file."""
        np.save("transform.npy", state["T"])
        print("Saved 4x4 transform to transform.npy\n", state["T"])

    def center_view(vis):
        """Resets the camera view to center the point clouds."""
        vis.reset_view_point(True)

    # ---- Visualizer & callbacks ----
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(window_name="Open3D PointCloud Align", width=1280, height=800)
    vis.add_geometry(state["tgt_draw"])
    vis.add_geometry(state["src_draw"])
    update_title(vis)

    # Key helpers
    def dtrans(dx=0, dy=0, dz=0):
        """Returns a callback function for translation."""

        def _cb(vis):
            apply_delta_transform(vis, dt=np.array([dx, dy, dz]))
            return False

        return _cb

    def drot(axis, deg):
        """Returns a callback function for rotation."""

        def _cb(vis):
            apply_delta_transform(vis, dR=rot_axis(axis, deg))
            return False

        return _cb

    def step_inc(vis, sign):
        """Increases or decreases the step sizes for translation and rotation."""
        if sign > 0:
            state["t_step"] *= 1.25
            state["r_step"] *= 1.25
        else:
            state["t_step"] /= 1.25
            state["r_step"] /= 1.25
        state["t_step"] = max(1e-4, state["t_step"])
        state["r_step"] = max(0.1, state["r_step"])
        vis.update_renderer()
        return False

    def icp_dist_inc(vis, sign):
        """Increases or decreases the ICP max correspondence distance."""
        if sign > 0:
            state["icp_dist"] *= 1.25
        else:
            state["icp_dist"] /= 1.25
        state["icp_dist"] = max(1e-4, state["icp_dist"])
        print(f"ICP max distance: {state['icp_dist']:.4f}")
        vis.update_renderer()
        return False

    def toggle_icp_type(vis):
        """Toggles between point-to-plane and point-to-point ICP."""
        state["icp_point_to_plane"] = not state["icp_point_to_plane"]
        print(
            f"ICP type: {'Point-to-Plane' if state['icp_point_to_plane'] else 'Point-to-Point'}"
        )
        vis.update_renderer()
        return False

    # Bind keys
    # Movement (uses current t_step)
    def bind_move(key, dx=0, dy=0, dz=0):
        vis.register_key_callback(
            ord(key),
            lambda v: dtrans(
                dx * state["t_step"], dy * state["t_step"], dz * state["t_step"]
            )(v),
        )

    # Rotation (uses current r_step)
    def bind_rot(key, axis, sign):
        vis.register_key_callback(
            ord(key), lambda v: drot(axis, sign * state["r_step"])(v)
        )

    bind_move("A", dx=-1)
    bind_move("D", dx=+1)
    bind_move("Q", dy=+1)
    bind_move("E", dy=-1)
    bind_move("W", dz=+1)
    bind_move("S", dz=-1)

    bind_rot("I", "x", +1)
    bind_rot("K", "x", -1)
    bind_rot("J", "y", +1)
    bind_rot("L", "y", -1)
    bind_rot("U", "z", +1)
    bind_rot("O", "z", -1)

    vis.register_key_callback(ord("["), lambda v: step_inc(v, -1))
    vis.register_key_callback(ord("]"), lambda v: step_inc(v, +1))
    vis.register_key_callback(ord(";"), lambda v: icp_dist_inc(v, -1))
    vis.register_key_callback(ord("'"), lambda v: icp_dist_inc(v, +1))

    vis.register_key_callback(ord("P"), toggle_icp_type)
    vis.register_key_callback(ord("T"), run_icp)
    vis.register_key_callback(ord("R"), reset_transform)
    vis.register_key_callback(ord("M"), save_transform)
    vis.register_key_callback(ord("C"), center_view)

    def help_callback(vis):
        print_help()
        return False

    vis.register_key_callback(ord("H"), help_callback)

    vis.run()
    vis.destroy_window()

    return state["T"]


def main():
    """Parses command-line arguments and runs the point cloud alignment."""
    parser = argparse.ArgumentParser(
        description="Minimal interactive point cloud alignment with ICP (Open3D)."
    )
    parser.add_argument("--src", required=True, help="Path to source .pcd (moving)")
    parser.add_argument("--tgt", required=True, help="Path to target .pcd (fixed)")
    parser.add_argument(
        "--voxel_vis",
        type=float,
        default=0.02,
        help="Visualization voxel size (downsample)",
    )
    parser.add_argument(
        "--voxel_icp", type=float, default=0.02, help="ICP voxel size (downsample)"
    )
    parser.add_argument(
        "--icp_max_dist",
        type=float,
        default=0.05,
        help="ICP max correspondence distance",
    )
    args = parser.parse_args()

    T = align_pointclouds(
        args.src,
        args.tgt,
        voxel_vis=args.voxel_vis,
        voxel_icp=args.voxel_icp,
        icp_max_dist=args.icp_max_dist,
    )
    # Also print the final transform for convenience
    np.set_printoptions(precision=5, suppress=True)
    print("\nFinal transform (source->target):\n", T)


if __name__ == "__main__":
    main()
