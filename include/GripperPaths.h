// Copyright (c) Intrinsic Innovation LLC
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#ifndef GRIPPER_PATHS_H
#define GRIPPER_PATHS_H


#include <vector>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

/**
 * @brief GripperPaths generates gripper paths for a robotic system.
 */
class GripperPaths {
public:
    /**
     * @brief Constructor for GripperPaths.
     *
     * @param size The size of the gripper in mm. Default is 10.0mm.
     * @param step The step size for each gripper movement along the passed directions, in mm.
     * @param z_tolerance The z tolerance.
     */
    GripperPaths(double size, double step, double z_tolerance);

    /**
     * @brief Generate the gripper paths.
     *
     * @param axis_aligned_bounding_box The bounding box of the point cloud (type: open3d::geometry::AxisAlignedBoundingBox).
     * @return A vector of vectors containing Eigen::Vector2d points that represent the gripper paths
     *         defined as the center point of the gripper in each descent path.
     */
    std::vector<std::vector<Eigen::Array2d>> GeneratePaths(
        const open3d::geometry::AxisAlignedBoundingBox& axis_aligned_bounding_box);
        
public:
    double size;        ///< Gripper size in mm
    double step;        ///< Step size in mm
    double z_tolerance; ///< Z tolerance
};

#endif // GRIPPER_PATHS_HPP