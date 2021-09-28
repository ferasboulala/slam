#pragma once

#include "pose.h"

#include <opencv2/opencv.hpp>

#include <vector>

namespace slam
{
class Lidar
{
public:
    Lidar(double start, double stop, double max_dist, double stddev,
          int n_rays);
    ~Lidar() = default;

    std::vector<Pose> scan(const cv::Mat &map, const Pose &pose) const;

    double start;
    double stop;
    double max_dist;
    double stddev;
    int n_rays;
};

}  // namespace slam