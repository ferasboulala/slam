#pragma once

#include <tuple>

#include <opencv2/opencv.hpp>

#include "pose.h"

namespace slam
{
Pose raycast(const cv::Mat& map, const Pose& pose, double max_distance,
             double step_size = 0.5);
Pose raycast_mapping(const cv::Mat& map, const Pose& pose, double distance, double max_distance,
             double step_size = 0.5);

double measurement_model_beam(double distance, double stddev,
                              Particle &particle, cv::Mat& occupied, cv::Mat& free,
                              Box &box, double max_distance, double stddev, double step_size = 0.5);

double measurement_model_beam_score(double distance, double stddev,
                              Particle &particle, cv::Mat& occupied, cv::Mat& free,
                              Box &box, double max_distance, double step_size = 0.5);

}  // namespace slam
