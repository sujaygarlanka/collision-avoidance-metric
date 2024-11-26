import open3d as o3d
import numpy as np

# Load a point cloud from a file (e.g., PLY, PCD, or XYZ format)
point_cloud = o3d.io.read_point_cloud("data_qp.ply")  # Replace with your file path

coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=10, origin=[0, 0, 0]  # Scale of the coordinate axes  # Origin of the axes
)


# Visualize the point cloud
o3d.visualization.draw_geometries([coordinate_frame, point_cloud], window_name="Point Cloud Viewer")
