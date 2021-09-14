#pragma once

#include "pose.h"
#include "lidar.h"

#include <Eigen/Dense>

#include <vector>

namespace slam
{
class MCL
{
public:
    MCL(const Eigen::MatrixXf& map, int n_particles, double alpha_fast = 0.9,
        double alpha_slow = 0.1);
    ~MCL() = default;

    void predict(const Odometry &odom);
    void update(const Lidar &lidar, const std::vector<double> &scans, const Eigen::MatrixXf& map);

    Pose average_pose() const;

private:
    void reset_particles();

public:
    std::vector<Particle> particles;

private:
    Eigen::MatrixXf m_map;
    double m_omega_fast;
    double m_omega_slow;
    double m_alpha_fast;
    double m_alpha_slow;
};

} // namespace slam
