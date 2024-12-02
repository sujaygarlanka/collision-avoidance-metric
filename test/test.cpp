#include "GripperPaths.h"

int main(int argc, char **argv)
{

    GripperPaths gripper_paths(10.0, 1.0, 0.1);
    Eigen::Vector3d min_bound(0.0, 0.0, 0.0);
    Eigen::Vector3d max_bound(1000.0, 1000.0, 1000.0);
    open3d::geometry::AxisAlignedBoundingBox axisAlignedBoundingBox(min_bound, max_bound);
    std::vector<std::vector<Eigen::Array2d>> paths = gripperPaths.GeneratePaths(axisAlignedBoundingBox);
    std::cout << "Number of paths: " << paths.size() << std::endl;

    return 0;
}
