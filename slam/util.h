#pragma once

#include "pose.h"

#include <Eigen/Dense>

#include <tuple>

namespace slam
{
double pdf_normal_distribution(double stddev, double x);
double pdf_normal_distribution_clamp(double stddev, double x,
                                     double multiple_stddev = 4);
double pdf_triangular_distribution(double stddev, double x);
double sample_normal_distribution(double variance);
double sample_triangular_distribution(double variance);
double normalize_angle(double angle);
std::tuple<int, int> pose_to_image_coordinates(const Eigen::MatrixXf& map, const Pose &pose);

} // namespace slam
