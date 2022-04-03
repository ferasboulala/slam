#pragma once

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <tuple>

#include "pose.h"
#include "util.h"

namespace slam
{
template <typename T>
Pose raycast(const cv::Mat& map,
             const Pose& pose,
             double max_distance_squared,
             double step_size = 0.5);

Pose raycast_mapping(Particle& particle,
                     double z_squared,
                     double z_max_squared,
                     double step_size = 1);

double measurement_model_beam(double distance_squared,
                              double stddev,
                              Particle& particle,
                              double max_distance_squared,
                              double step_size = 1);

}  // namespace slam
