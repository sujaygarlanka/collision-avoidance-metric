import open3d as o3d
import numpy as np

# xyz = np.random.rand(100, 3)
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(xyz)
# o3d.io.write_point_cloud("./data.ply", pcd)

# num_points = 100
# num_lines = 10
# z_space = 10
# final_line = []
# for i in range(num_lines):
#     x = np.linspace(0, 600, num_points)  # Linearly spaced values for X
#     y = np.full_like(x, 100)   # Y values are constant (0)
#     z = np.full_like(x, i * z_space)  # Z values are constant (0)
#     line_points = np.column_stack((x, y, z)).tolist()
#     final_line += line_points

#     x = np.linspace(601, 1000, num_points)  # Linearly spaced values for X
#     y = np.full_like(x, 0.0)  # Y values are constant (0)
#     z = np.full_like(x, i * z_space)  # Z values are constant (0)
#     line_points = np.column_stack((x, y, z)).tolist()
#     final_line += line_points

# xyz = np.array(final_line)
# print(xyz)
# print(xyz.shape)
x = np.linspace(0, 1000, 5)  # 10 points along X
y = np.linspace(0, 1000, 5)  # 15 points along Y
z = np.linspace(1, 1000, 5)  # 20 points along Z

# Create a meshgrid
X, Y, Z = np.meshgrid(x, y, z)

# Combine into a single array of shape (N, 3)
values = [1000, 0]
xyz = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T
xyz = xyz[np.any(np.isin(xyz, values), axis=1)]
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
o3d.io.write_point_cloud("./data_qp.ply", pcd)
