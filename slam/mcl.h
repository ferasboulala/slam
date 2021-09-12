#pragma once

#include "pose.h"

#include <vector>

namespace slam
{

class MCL
{
  public:
    MCL(int n_particles, double alpha_fast = 0.9, double alpha_slow = 0.1);
    ~MCL() = default;

  private:
    std::vector<Pose> m_points_poses;
    double m_omega_fast;
    double m_omega_slow;
    double m_alpha_fast;
    double m_alpha_slow;
};

} // namespace slam
