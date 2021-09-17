#include "lidar.h"
#include "raycast.h"
#include "util.h"

#include <cmath>

namespace slam
{
Lidar::Lidar(double start, double stop, double max_dist, double stddev,
             int n_rays)
    : start(start),
      stop(stop),
      max_dist(max_dist),
      stddev(stddev),
      n_rays(n_rays)
{
}

std::vector<Pose> Lidar::scan(const cv::Mat& map, const Pose& pose) const
{
    const double range = stop - start;
    const double step = range / n_rays;

    std::vector<Pose> scans;
    scans.reserve(n_rays);

    Pose raycast_pose = pose;
    raycast_pose.theta -= range / 2;
    for (int i = 0; i < n_rays; ++i)
    {
        scans.push_back(raycast<int>(map, raycast_pose, max_dist));
        raycast_pose.theta += step;
    }

    return scans;
}

}  // namespace slam