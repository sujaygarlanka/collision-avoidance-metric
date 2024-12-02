#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <open3d/Open3D.h>
#include <iostream>

int main(int argc, char **argv)
{
    // Load the input image
    std::string imagePath = "/home/sujay/Code/collision-avoidance-metric/lambo.jpg";
    cv::Mat image = cv::imread(imagePath, cv::IMREAD_COLOR);

    // Check if the image is loaded successfully
    if (image.empty())
    {
        std::cerr << "Error: Could not open or find the image!" << std::endl;
        return -1;
    }

    // Convert the image to grayscale
    cv::Mat grayImage;
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    std::cout << "Image size: " << image.size() << std::endl;

    open3d::geometry::PointCloud pcd;
    pcd.points_.push_back(Eigen::Vector3d(1.0, 0.0, 0.0));
    pcd.points_.push_back(Eigen::Vector3d(0.0, 1.0, 0.0));
    pcd.points_.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));

    std::cout << "PointCloud has " << pcd.points_.size() << " points.\n";

    // Display the original and grayscale images
    cv::imshow("Original Image", image);
    cv::imshow("Grayscale Image", grayImage);

    // Wait for a key press indefinitely
    cv::waitKey(0);

    return 0;
}
