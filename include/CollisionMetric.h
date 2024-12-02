// Copyright (c) Intrinsic Innovation LLC
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#include <Eigen/Dense>
#include <iostream>
#include <vector>

/**
 * @brief Struct to store results for collision metric computation.
 */
struct CollisionMetricsResults {
    double fp;                       ///< False Positive collisions
    double fn;                       ///< False Negative collisions
    int total_paths;                 ///< Total gripper paths
    Eigen::ArrayXd fpmap;            ///< False Positive collisions map
    Eigen::ArrayXd fnmap;            ///< False Negative collisions map

    /**
     * @brief Constructor to initialize the struct.
     * 
     * @param fp False Positive collisions
     * @param fn False Negative collisions
     * @param total_paths Total gripper paths
     * @param fpmap False Positive collisions map
     * @param fnmap False Negative collisions map
     */
    CollisionMetricsResults(double fp, double fn, int total_paths,
                            const Eigen::ArrayXd& fpmap, const Eigen::ArrayXd& fnmap)
        : fp(fp), fn(fn), total_paths(total_paths), fpmap(fpmap), fnmap(fnmap) {}

    /**
     * @brief Print the collision metrics for debugging or display.
     */
    void Print() const {
        std::cout << "CollisionMetricsResults:\n";
        std::cout << "  False Positives (fp): " << fp << "\n";
        std::cout << "  False Negatives (fn): " << fn << "\n";
        std::cout << "  Total Paths: " << total_paths << "\n";
        std::cout << "  False Positive Map (fpmap): " << fpmap.transpose() << "\n";
        std::cout << "  False Negative Map (fnmap): " << fnmap.transpose() << "\n";
    }
};
