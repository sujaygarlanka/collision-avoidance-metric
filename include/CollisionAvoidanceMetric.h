#include <vector>
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include "GripperPaths.h"
#include "CollisionMetric.h"

class CollisionAvoidanceMetric {
    public:
        CollisionAvoidanceMetric(std::vector<double> gripper_z_tolerances, std::vector<Eigen::Vector3d> directions, int outlier_threshold, double gripper_size, int gripper_step);
        std::unordered_map<double, std::unordered_map<std::string, double>> ComputeCollisionMetrics(open3d::geometry::PointCloud query_point_cloud, open3d::geometry::PointCloud gt_point_cloud);
    
    public:
        std::vector<double> gripper_z_tolerances; 
        std::vector<Eigen::Vector3d> directions;  
        int outlier_threshold;                    
        double gripper_size;                   
        int gripper_step;               

    private:
        CollisionMetricsResults ComputeCollisionForGripperDirection(open3d::geometry::PointCloud query_point_cloud, open3d::geometry::PointCloud gt_point_cloud, GripperPaths gripper_paths, Eigen::Vector3d direction);
        std::vector<double> ComputeCollisionMetricForZTolerance(open3d::geometry::PointCloud query_point_cloud, open3d::geometry::PointCloud gt_point_cloud, double gripper_z_tolerance);

}