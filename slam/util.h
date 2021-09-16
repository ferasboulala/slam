#pragma once

#include "pose.h"

#include <opencv2/opencv.hpp>

#include <tuple>

namespace slam
{
double pdf_normal_distribution(double stddev, double x);
double pdf_normal_distribution_clamp(double stddev, double x,
                                     double multiple_stddev = 4);
double pdf_triangular_distribution(double stddev, double x);
double sample_normal_distribution(double stddev);
double sample_triangular_distribution(double stddev);
double normalize_angle(double angle);

inline std::tuple<int, int> pose_to_image_coordinates(const cv::Mat& map,
                                               const Pose& pose)
{
    return std::tuple<int, int>(map.rows - pose.y - 1, pose.x);
}

inline bool within_boundaries(const cv::Mat& map, const int i, const int j)
{
    return i < map.rows && j < map.cols && i >= 0 && j >= 0;
}

}  // namespace slam
