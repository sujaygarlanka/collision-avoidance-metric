// Copyright (c) Intrinsic Innovation LLC
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#include "GripperPaths.h"
#include <vector>
#include <Eigen/Dense>
#include <cmath> // For std::ceil

// Constructor definition
GripperPaths::GripperPaths(double size, double step, double z_tolerance)
    : size(size), step(step), z_tolerance(z_tolerance) {}

// GeneratePaths function definition
std::vector<std::vector<Eigen::Array2d>> GripperPaths::GeneratePaths(
    const open3d::geometry::AxisAlignedBoundingBox& axis_aligned_bounding_box) {

    // Get the bounding box's minimum and maximum bounds
    auto min_bound = axis_aligned_bounding_box.GetMinBound();
    auto max_bound = axis_aligned_bounding_box.GetMaxBound();

    // Initialize the starting point
    auto init_point = Eigen::Array2d(min_bound[0] + size / 2, min_bound[1] + size / 2);

    // Generate linspace for x and y
    Eigen::ArrayXd x = Eigen::ArrayXd::LinSpaced(
        static_cast<int>(std::ceil((max_bound[0] - init_point[0]) / step)),
        init_point[0],
        max_bound[0]
    );

    Eigen::ArrayXd y = Eigen::ArrayXd::LinSpaced(
        static_cast<int>(std::ceil((max_bound[1] - init_point[1]) / step)),
        init_point[1],
        max_bound[1]
    );

    // Initialize paths
    std::vector<std::vector<Eigen::Array2d>> paths(
        y.size(),
        std::vector<Eigen::Array2d>(x.size())
    );

    // Populate paths
    for (int i = 0; i < y.size(); ++i) {
        for (int j = 0; j < x.size(); ++j) {
            paths[i][j] = Eigen::Array2d(x[j], y[i]);
        }
    }

    return paths;
}

