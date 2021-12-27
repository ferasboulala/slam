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
Pose raycast(const cv::Mat& map, const Pose& pose, double max_distance, double step_size = 0.5);

void raycast_mapping(Particle& particle, double z, double z_max, double step_size = 0.5);

double measurement_model_beam(
    double distance, double stddev, Particle& particle, double max_distance, double step_size = 0.5);

double measurement_model_beam_score(double distance,
                                    double stddev,
                                    Particle& particle,
                                    cv::Mat& occupied,
                                    cv::Mat& free,
                                    Box& box,
                                    double max_distance,
                                    double step_size = 0.5);

}  // namespace slam
