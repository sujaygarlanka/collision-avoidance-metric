#include "PointCloud.h"
#include <optional>
#include <cmath>
#include <GripperPaths.h>

double _detect_collision_point(Eigen::MatrixXd &point_cloud, double gripper_size, int outlier_threshold){
    // Detect collision point
    // Return collision point
    Eigen::VectorXd flattened = Eigen::Map<Eigen::VectorXd>(point_cloud.data(), point_cloud.size());
    std::vector<double> z_points(flattened.data(), flattened.data() + flattened.size());

    // Sort the std::vector
    std::sort(z_points.begin(), z_points.end());
    int outliers = 0;
    for (int i = 0; i < z_points.size(); i++){
        if (std::isnan(z_points[i])){
            outliers++;
        }
    }
    if (outliers <= outlier_threshold){
        return std::numeric_limits<double>::quiet_NaN();
    }

    Eigen::VectorXd z_points_sorted = Eigen::Map<Eigen::VectorXd>(z_points.data(), z_points.size());
    Eigen::VectorXd diff = z_points_sorted.segment(outlier_threshold, z_points_sorted.size()) - z_points_sorted.segment(0, z_points_sorted.size() - outlier_threshold);
    // Find the first collision point
    for (int i = 0; i < diff.size(); ++i) {
        if (diff[i] < gripper_size) {
            return z_points_sorted[i]; // Return the corresponding z-coordinate
        }
    }

    // No collision detected
    return std::numeric_limits<double>::quiet_NaN();
}

PointCloud::PointCloud(const open3d::geometry::PointCloud& point_cloud){
    point_cloud_ = point_cloud;
}

void PointCloud::RotateAndUpdateAABB(Eigen::Vector3d direction, std::optional<open3d::geometry::PointCloud> &check_point_cloud){
    // Rotate the point cloud
    // Update the axis-aligned bounding box
    auto mesh = open3d::geometry::TriangleMesh::CreateCoordinateFrame();
    auto rotation_matrix = mesh->GetRotationMatrixFromXYZ(direction);
    rotated_point_cloud_ = open3d::geometry::PointCloud(point_cloud_);
    rotated_point_cloud_.Rotate(rotation_matrix, Eigen::Vector3d(0.0, 0.0, 0.0));

    if (check_point_cloud.has_value()){
        aabb_ = check_point_cloud->GetAxisAlignedBoundingBox();
        aabb_.min_bound_ = aabb_.GetMinBound() - Eigen::Vector3d(0.0, 0.0, 100.0);
        rotated_point_cloud_ = *rotated_point_cloud_.Crop(aabb_);
    }
    else{
        aabb_ = check_point_cloud->GetAxisAlignedBoundingBox();
        aabb_.min_bound_ = aabb_.GetMinBound() - Eigen::Vector3d(0.0, 0.0, 100.0);
    }
    return;
}

Eigen::Matrix2d PointCloud::GenerateCollisionMap(GripperPaths &gripper_paths, int outlier_threshold){
    // Generate collision map
    // Check for collision
    // Return collision map

    std::vector<std::vector<Eigen::Array2d>> paths = gripper_paths.GeneratePaths(aabb_);
    Eigen::Matrix2d collision_map = Eigen::Matrix2d::Zero(paths.size(), paths[0].size());
    std::vector<Eigen::Vector3d> points = rotated_point_cloud.points_;
    std::vector<Eigen::Vector3d> points_rounded;
    // Eigen::MatrixXd points_matrix(points.size(), 3);
    // Copy data from the std::vector into the Eigen matrix
    // for (size_t i = 0; i < points.size(); ++i) {
    //     points_matrix.row(i) = points[i];
    // }

    Eigen::Vector3d aabb_extent = aabb_.GetExtent();
    Eigen::Vector3d abb_min_bound = aabb_.GetMinBound();

    Eigen::MatrixXd voxel_grid = Eigen::MatrixXd::Zero(aabb_extent[1] + 1, aabb_extent[0] + 1);
    // Eigen::MatrixXd points_rounded = points_matrix.rowwise() - abb_min_bound.transpose();
    // points_rounded.unaryExpr([](double x) { return std::round(x); });


    for (int i = 0; i < points.size(); i++){
        // Eigen::Vector3d rounded = std::round(points[i] - abb_min_bound);
        Eigen::Vector3d rounded = points[i] - abb_min_bound;
        rounded.unaryExpr([](double x) { return std::round(x); });
        points_rounded.push_back(rounded);
    }

    std::sort(points_rounded.begin(), points_rounded.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return a.z() > b.z(); // Compare based on z-component
    });

    for (int i = 0; i < points_rounded.size(); i++){
        int val = static_cast<int>(points_rounded[i].z());
        voxel_grid(static_cast<int>(points_rounded[i].y()), static_cast<int>(points_rounded[i].x())) = val;
    }

    voxel_grid = voxel_grid.unaryExpr([](double x) {
        return (x == 0) ? std::numeric_limits<double>::quiet_NaN() : x;
    });

    for (int y = 0; y < paths.size(); y++) {
        for (int x = 0; x < paths[y].size(); x++) {
            Eigen::Array2d path = paths[y][x];
            int min_x = static_cast<int>(std::round(path[1]- abb_min_bound[1] - gripper_paths.size / 2));
            int max_x = static_cast<int>(std::round(path[1] - abb_min_bound[1] + gripper_paths.size / 2));
            int min_y = static_cast<int>(std::round(path[0] - abb_min_bound[0] - gripper_paths.size / 2));
            int max_y = static_cast<int>(std::round(path[0] - abb_min_bound[0] + gripper_paths.size / 2));

            Eigen::MatrixXd pcd_crop = voxel_grid.block(min_x, min_y, max_x - min_x, max_y - min_y);
            double collision_point = _detect_collision_point(pcd_crop, gripper_paths.size, outlier_threshold);
            collision_map(y, x) = collision_point;
        }
    }

    collision_map = collision_map.unaryExpr([](double x) {
        return (x == 0) ? std::numeric_limits<double>::quiet_NaN() : x;
    });
    collision_map_ = collision_map;
    paths_ = paths;
    return collision_map;
}

Eigen::Vector3d PointCloud::ConvertTensorToVector3d(open3d::core::Tensor tensor){
    // Convert tensor to Eigen::Vector3d
    return Eigen::Vector3d(tensor[0].Item<double>(), tensor[1].Item<double>(), tensor[2].Item<double>());
}