#pragma once

#include <tuple>

#include <Eigen/Dense>

#include "pose.h"

namespace slam
{

Pose raycast(const Eigen::MatrixXf& map, const Pose& pose, double max_distance,
             double step_size = 0.2);

} // namespace slam
