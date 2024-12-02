
#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include <optional>

class PointCloud {
    public:
        PointCloud(const open3d::geometry::PointCloud&);

        void RotateAndUpdateAABB(Eigen::Vector3d, std::optional<open3d::geometry::PointCloud>&);
        Eigen::Matrix2d GenerateCollisionMap(GripperPaths &gripper_paths, int outlier_threshold);
        ~PointCloud();

    public:
        open3d::geometry::PointCloud rotated_point_cloud;

    private:
        open3d::geometry::PointCloud point_cloud_;
        open3d::geometry::AxisAlignedBoundingBox aabb_;
        Eigen::MatrixXd collision_map_;
        std::vector<std::vector<Eigen::Array2d>> paths_;

        Eigen::Vector3d ConvertTensorToVector3d(open3d::core::Tensor);  
}