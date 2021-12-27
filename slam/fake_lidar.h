#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "pose.h"

namespace slam
{
class FakeLidar
{
public:
    FakeLidar(double start, double stop, double max_dist, double stddev, int n_rays);
    ~FakeLidar() = default;

    std::vector<Pose> scan(const cv::Mat& map, const Pose& pose) const;

    double start;
    double stop;
    double max_dist;
    double stddev;
    int n_rays;
};

}  // namespace slam
