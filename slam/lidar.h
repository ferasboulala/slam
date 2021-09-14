#pragma once

#include "pose.h"

#include <Eigen/Dense>

#include <vector>

namespace slam
{
class Lidar
{
public:
    Lidar(double start, double stop, double max_dist, double stddev,
          int n_rays);
    ~Lidar() = default;

    std::vector<Pose> scan(const Eigen::MatrixXf& map, const Pose& pose) const;

    double start;
    double stop;
    double max_dist;
    double stddev;
    int n_rays;
};

}  // namespace slam