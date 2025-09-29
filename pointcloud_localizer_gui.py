"""
Airborne Sensor Localizer – Minimal Open3D GUI (MVP)
----------------------------------------------------

Features in this MVP:
- Load a PRIOR MAP point cloud (PCD/PLY/PTS/XYZ/OBJ, etc. via Open3D)
- Load a LIVE SCAN point cloud (e.g., from the airborne LIDAR or a saved sweep)
- Downsample + normals estimation (configurable voxel size)
- Global coarse alignment (RANSAC on FPFH)
- Local refinement (point-to-plane ICP / Generalized ICP)
- Visualize both clouds, show the estimated transform, and export it
- Parameter panel with sliders/buttons; background registration thread

Dependencies:
  pip install open3d numpy

Notes:
- This is file-based to start. Live ROS2 streaming will be added in a follow-up module.
- Stereo integration (triangulated sparse cloud / depth fusion) will be a later module.
- Assumes map & scan are in the same metric units.
"""

from __future__ import annotations
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import open3d as o3d

# --------------------------- Registration Backend --------------------------- #

@dataclass
class RegParams:
    voxel_size: float = 0.5              # meters
    ransac_distance: float = 1.5         # meters (max correspondence distance for RANSAC)
    icp_distance: float = 0.5            # meters (max correspondence distance for ICP)
    use_generalized_icp: bool = False    # toggle GICP vs. point-to-plane ICP
    normal_radius_factor: float = 2.0    # radius for normal/FPFH = factor * voxel_size
    fpfh_radius_factor: float = 5.0
    ransac_n: int = 4                    # number of points per RANSAC correspondence set
    ransac_iterations: int = 200000      # total samples (Open3D counts)
    ransac_validations: int = 1000       # correspondence checks


def _estimate_normals(pcd: o3d.geometry.PointCloud, radius: float) -> None:
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=50)
    )
    pcd.normalize_normals()


def preprocess_point_cloud(pcd: o3d.geometry.PointCloud, voxel: float, normal_radius: float,
                           fpfh_radius: float) -> Tuple[o3d.geometry.PointCloud, o3d.pipelines.registration.Feature]:
    pcd_ds = pcd.voxel_down_sample(voxel_size=voxel)
    if len(pcd_ds.points) == 0:
        raise ValueError("Downsampled cloud is empty. Try a smaller voxel size.")
    _estimate_normals(pcd_ds, radius=normal_radius)
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_ds,
        o3d.geometry.KDTreeSearchParamHybrid(radius=fpfh_radius, max_nn=100),
    )
    return pcd_ds, fpfh


def global_registration_ransac(source: o3d.geometry.PointCloud,
                               target: o3d.geometry.PointCloud,
                               params: RegParams) -> o3d.pipelines.registration.RegistrationResult:
    voxel = params.voxel_size
    normal_radius = params.normal_radius_factor * voxel
    fpfh_radius = params.fpfh_radius_factor * voxel

    src_ds, src_fpfh = preprocess_point_cloud(source, voxel, normal_radius, fpfh_radius)
    tgt_ds, tgt_fpfh = preprocess_point_cloud(target, voxel, normal_radius, fpfh_radius)

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_ds, tgt_ds, src_fpfh, tgt_fpfh,
        mutual_filter=True,
        max_correspondence_distance=params.ransac_distance,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=params.ransac_n,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(params.ransac_distance),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(params.ransac_iterations, params.ransac_validations),
    )
    return result


def refine_registration(source: o3d.geometry.PointCloud,
                        target: o3d.geometry.PointCloud,
                        init_T: np.ndarray,
                        params: RegParams) -> o3d.pipelines.registration.RegistrationResult:
    _estimate_normals(target, radius=params.normal_radius_factor * params.voxel_size)
    _estimate_normals(source, radius=params.normal_radius_factor * params.voxel_size)

    if params.use_generalized_icp:
        estimation = o3d.pipelines.registration.TransformationEstimationForGeneralizedICP()
    else:
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()

    result = o3d.pipelines.registration.registration_icp(
        source, target, params.icp_distance, init_T, estimation,
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
    )
    return result


# --------------------------- GUI / Application ----------------------------- #

class LocalizerApp:
    def __init__(self):
        o3d.visualization.gui.Application.instance.initialize()
        self.window = o3d.visualization.gui.Application.instance.create_window(
            "Airborne Sensor Localizer (MVP)", 1400, 900
        )

        self.scene = o3d.visualization.gui.SceneWidget()
        self.scene.scene = o3d.visualization.rendering.Open3DScene(self.window.renderer)
        self.scene.scene.set_background([0, 0, 0, 1])
        self.scene.set_on_key(self._on_key)

        em = o3d.visualization.gui.Theme().font_size
        self.panel = o3d.visualization.gui.Vert(0, o3d.visualization.gui.Margins(10, 10, 10, 10))

        # Load buttons
        self.btn_load_map = o3d.visualization.gui.Button("Load Map Cloud")
        self.btn_load_scan = o3d.visualization.gui.Button("Load Scan Cloud")
        self.btn_save_T = o3d.visualization.gui.Button("Export Transform…")
        self.btn_run = o3d.visualization.gui.Button("Run Align (RANSAC+ICP)")
        self.btn_reset_view = o3d.visualization.gui.Button("Reset View")

        # Params
        self.voxel_slider = o3d.visualization.gui.Slider(o3d.visualization.gui.Slider.DOUBLE)
        self.voxel_slider.set_limits(0.05, 3.0)
        self.voxel_slider.double_value = 0.5

        self.ransac_dist_slider = o3d.visualization.gui.Slider(o3d.visualization.gui.Slider.DOUBLE)
        self.ransac_dist_slider.set_limits(0.1, 5.0)
        self.ransac_dist_slider.double_value = 1.5

        self.icp_dist_slider = o3d.visualization.gui.Slider(o3d.visualization.gui.Slider.DOUBLE)
        self.icp_dist_slider.set_limits(0.05, 2.0)
        self.icp_dist_slider.double_value = 0.5

        self.chk_gicp = o3d.visualization.gui.Checkbox("Use Generalized ICP")
        self.chk_gicp.checked = False

        # Status label
        self.status = o3d.visualization.gui.Label("Ready.")

        # Layout panel
        self.panel.add_child(self.btn_load_map)
        self.panel.add_child(self.btn_load_scan)
        self.panel.add_fixed(0.25 * em)
        self.panel.add_child(o3d.visualization.gui.Label("Voxel size (m)"))
        self.panel.add_child(self.voxel_slider)
        self.panel.add_child(o3d.visualization.gui.Label("RANSAC max dist (m)"))
        self.panel.add_child(self.ransac_dist_slider)
        self.panel.add_child(o3d.visualization.gui.Label("ICP max dist (m)"))
        self.panel.add_child(self.icp_dist_slider)
        self.panel.add_child(self.chk_gicp)
        self.panel.add_fixed(0.5 * em)
        self.panel.add_child(self.btn_run)
        self.panel.add_child(self.btn_save_T)
        self.panel.add_child(self.btn_reset_view)
        self.panel.add_fixed(0.5 * em)
        self.panel.add_child(self.status)

        # Split view
        self.hsplit = o3d.visualization.gui.Horiz(0, o3d.visualization.gui.Margins(0, 0, 0, 0))
        self.hsplit.add_child(self.scene, 3)
        self.hsplit.add_child(self.panel, 1)

        self.window.add_child(self.hsplit)

        # Connect callbacks
        self.btn_load_map.set_on_clicked(self._on_load_map)
        self.btn_load_scan.set_on_clicked(self._on_load_scan)
        self.btn_run.set_on_clicked(self._on_run_align)
        self.btn_save_T.set_on_clicked(self._on_save_transform)
        self.btn_reset_view.set_on_clicked(self._on_reset_view)

        # State
        self.map_pcd: Optional[o3d.geometry.PointCloud] = None
        self.scan_pcd: Optional[o3d.geometry.PointCloud] = None
        self.current_T: np.ndarray = np.eye(4)
        self.map_name = "map"
        self.scan_name = "scan"

        # Rendering materials
        self.mat_map = o3d.visualization.rendering.MaterialRecord()
        self.mat_map.shader = "defaultLit"
        self.mat_map.base_color = (0.6, 0.6, 0.6, 1.0)
        self.mat_scan = o3d.visualization.rendering.MaterialRecord()
        self.mat_scan.shader = "defaultLit"
        self.mat_scan.base_color = (0.1, 0.8, 0.2, 1.0)

    # ------------------------- Helper / UI methods ------------------------- #

    def _set_status(self, msg: str):
        self.status.text = msg
        self.window.set_needs_layout()

    def _clear_scene(self):
        self.scene.scene.clear_geometry()

    def _update_scene(self):
        self._clear_scene()
        if self.map_pcd is not None:
            self.scene.scene.add_geometry(self.map_name, self.map_pcd, self.mat_map)
        if self.scan_pcd is not None:
            transformed = o3d.geometry.PointCloud(self.scan_pcd)
            transformed.transform(self.current_T)
            self.scene.scene.add_geometry(self.scan_name, transformed, self.mat_scan)
        if self.scene.scene.scene:  # set some lighting
            self.scene.setup_camera(60.0, self.scene.scene.bounding_box, self.scene.scene.bounding_box.get_center())

    def _on_key(self, event):
        # Simple key: space to center view
        if event.key == o3d.visualization.gui.Key.SPACE:
            self._on_reset_view(None)
            return o3d.visualization.gui.Widget.EventCallbackResult.HANDLED
        return o3d.visualization.gui.Widget.EventCallbackResult.IGNORED

    # ------------------------------ Callbacks ------------------------------ #

    def _on_load_map(self):
        dlg = o3d.visualization.gui.FileDialog(o3d.visualization.gui.FileDialog.OPEN, "Load map point cloud", self.window.theme)
        dlg.add_filter("Point clouds", ".ply .pcd .pts .xyz .xyzn .xyzrgb .obj")
        dlg.set_on_done(lambda path: self._load_pcd_file(path, is_map=True))
        dlg.set_on_cancel(lambda: None)
        self.window.show_dialog(dlg)

    def _on_load_scan(self):
        dlg = o3d.visualization.gui.FileDialog(o3d.visualization.gui.FileDialog.OPEN, "Load scan point cloud", self.window.theme)
        dlg.add_filter("Point clouds", ".ply .pcd .pts .xyz .xyzn .xyzrgb .obj")
        dlg.set_on_done(lambda path: self._load_pcd_file(path, is_map=False))
        dlg.set_on_cancel(lambda: None)
        self.window.show_dialog(dlg)

    def _load_pcd_file(self, path: str, is_map: bool):
        self.window.close_dialog()
        try:
            pcd = o3d.io.read_point_cloud(path)
            if pcd.is_empty():
                raise RuntimeError("Loaded point cloud is empty")
            if is_map:
                self.map_pcd = pcd
                self.map_name = f"map:{path.split('/')[-1]}"
            else:
                self.scan_pcd = pcd
                self.scan_name = f"scan:{path.split('/')[-1]}"
            self._set_status(f"Loaded {'map' if is_map else 'scan'} cloud with {np.asarray(pcd.points).shape[0]:,} points")
            self._update_scene()
        except Exception as e:
            self._set_status(f"Error loading point cloud: {e}")
            traceback.print_exc()

    def _on_run_align(self):
        if self.map_pcd is None or self.scan_pcd is None:
            self._set_status("Load both map and scan clouds first.")
            return

        params = RegParams(
            voxel_size=self.voxel_slider.double_value,
            ransac_distance=self.ransac_dist_slider.double_value,
            icp_distance=self.icp_dist_slider.double_value,
            use_generalized_icp=self.chk_gicp.checked,
        )

        def worker():
            try:
                self._set_status("Running global registration (RANSAC)…")
                t0 = time.time()
                result_global = global_registration_ransac(self.scan_pcd, self.map_pcd, params)
                t1 = time.time()
                self._set_status(f"Global init done (fitness={result_global.fitness:.3f}, inlier_rmse={result_global.inlier_rmse:.3f}) → refining…")

                result_refine = refine_registration(self.scan_pcd, self.map_pcd, result_global.transformation, params)
                t2 = time.time()

                self.current_T = result_refine.transformation
                self._update_scene()

                self._set_status(
                    "Aligned. ICP fitness={:.3f}, RMSE={:.3f}. Times: RANSAC {:.1f}s, ICP {:.1f}s".format(
                        result_refine.fitness, result_refine.inlier_rmse, t1 - t0, t2 - t1
                    )
                )
            except Exception as e:
                self._set_status(f"Registration error: {e}")
                traceback.print_exc()

        threading.Thread(target=worker, daemon=True).start()

    def _on_save_transform(self):
        dlg = o3d.visualization.gui.FileDialog(o3d.visualization.gui.FileDialog.SAVE, "Save 4x4 transform (npz)", self.window.theme)
        dlg.add_filter("NumPy archive", ".npz")
        def _save(path: str):
            self.window.close_dialog()
            try:
                np.savez(path, T=self.current_T)
                self._set_status(f"Saved transform to {path}")
            except Exception as e:
                self._set_status(f"Failed to save transform: {e}")
        dlg.set_on_done(_save)
        dlg.set_on_cancel(lambda: None)
        self.window.show_dialog(dlg)

    def _on_reset_view(self, _):
        if self.scene.scene.bounding_box is not None:
            self.scene.setup_camera(60.0, self.scene.scene.bounding_box, self.scene.scene.bounding_box.get_center())


# ------------------------------- Entrypoint -------------------------------- #

def main():
    app = LocalizerApp()
    o3d.visualization.gui.Application.instance.run()


if __name__ == "__main__":
    main()
