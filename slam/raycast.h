#pragma once

#include <tuple>

#include <Eigen/Dense>

#include "pose.h"

namespace slam
{
Pose raycast(const Eigen::MatrixXf& map, const Pose& pose, double max_distance,
             double step_size = 0.5);

double measurement_model_beam(double distance, double stddev,
                              const Eigen::MatrixXf& map, const Pose& pose,
                              double max_distance, double step_size = 0.5);

double measurement_model_beam_score(double distance, double stddev,
                              const Eigen::MatrixXf& map, const Pose& pose,
                              double max_distance, double step_size = 0.5);

}  // namespace slam
