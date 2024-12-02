#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <limits> // For NaN

Eigen::Tensor<double, 3> xyToleranceMapGenerator(const Eigen::MatrixXd& map1, const Eigen::MatrixXd& map2) {
    int M = map1.rows();
    int N = map1.cols();

    Eigen::MatrixXd shiftedmap1 = Eigen::MatrixXd::Constant(M + 1, N + 1, std::numeric_limits<double>::quiet_NaN());
    Eigen::MatrixXd shiftedmap2 = Eigen::MatrixXd::Constant(M + 1, N + 1, std::numeric_limits<double>::quiet_NaN());

    // Copy map2 into appropriate regions of the padded maps
    shiftedmap1.block(0, 0, M, N) = map2; // Pad bottom and right
    shiftedmap2.block(1, 1, M, N) = map2; // Pad top and left

    // Initialize the result (MxNx5 tensor)
    Eigen::Tensor<double, 3> xy_tolerance_map(M, N, 5);

    // Compute the difference maps
    for (int i = 0; i < M; ++i) {
        for (int j = 0; j < N; ++j) {
            xy_tolerance_map(i, j, 0) = map1(i, j) - map2(i, j);                        // Direct difference
            xy_tolerance_map(i, j, 1) = map1(i, j) - shiftedmap1(i, j + 1);             // Shifted down and right
            xy_tolerance_map(i, j, 2) = map1(i, j) - shiftedmap1(i + 1, j);             // Shifted up and left
            xy_tolerance_map(i, j, 3) = map1(i, j) - shiftedmap2(i, j + 1);             // Shifted up and right
            xy_tolerance_map(i, j, 4) = map1(i, j) - shiftedmap2(i + 1, j);             // Shifted down and left
        }
    }

    return xy_tolerance_map;
}