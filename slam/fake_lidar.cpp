#include "fake_lidar.h"

#include <cmath>

#include "raycast.h"
#include "util.h"

namespace slam
{
FakeLidar::FakeLidar(double start, double stop, double max_dist, double stddev, int n_rays)
    : start(start), stop(stop), max_dist(max_dist), stddev(stddev), n_rays(n_rays)
{
}

std::vector<Pose> FakeLidar::scan(const cv::Mat& map, const Pose& pose) const
{
    const double range = stop - start;
    const double step = range / n_rays;

    std::vector<Pose> scans;
    scans.reserve(n_rays);

    Pose raycast_pose = pose;
    raycast_pose.theta -= range / 2;
    const double max_dist_squared = max_dist * max_dist;
    for (int i = 0; i < n_rays; ++i)
    {
        scans.push_back(raycast<int>(map, raycast_pose, max_dist_squared));
        raycast_pose.theta += step;
    }

    return scans;
}

}  // namespace slam
