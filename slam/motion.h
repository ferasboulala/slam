#pragma once

#include <array>

#include "pose.h"

namespace slam
{
Pose sample_motion_model_odometry(const Odometry& odom,
                                  const Pose& pose,
                                  std::array<double, 4> alphas = {0, 0, 0, 0});
Pose sample_motion_model_velocity(const Velocity& vel,
                                  const Pose& pose,
                                  double dt,
                                  std::array<double, 6> alphas = {0, 0, 0, 0, 0, 0});

}  // namespace slam
