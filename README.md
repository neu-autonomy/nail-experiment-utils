# NAIL Experiment Utilities

Small utility codes for field testing and demonstrations. Each directory is a standalone
set of scripts/tools. Within each directory, there is a README.md with more details about usage.

- **foxglove**: setup/launch foxglove studio with ROS 2 bridge on a payload
- **make_pcd**: from a payload with ouster lidar, record a pointcloud `.pcd` file and upload to a remote host
- **pcd_registration**: given two pointclouds as `.pcd` files, roughly align them manually and then run ICP to refine the alignment. Saves the resulting transform.
