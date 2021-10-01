#pragma once

#include "pose.h"

#include <limits>

namespace slam
{
class HybridAStar
{
public:
    HybridAStar(cv::Mat &mat, const Pose &A, const Pose &B, double v,
                double theta, unsigned theta_res = 10)
        : Planner(map, A, B), m_v(v), m_theta(theta)
    {
        m_distances = Cuboid(map.rows, Grid(map.cols, std::vector<double>(theta_res, std::numeric_limits<double>::max()));
    }

    bool pathfind(cv::Mat *canvas) { return false; }
    std::vector<Coordinate> recover_path() { return {}; }
private:
    double m_v;
    double m_theta;

    using Grid = std::vector<std::vector<double>>;
    using Cuboid = std::vector<Grid>;
    Cuboid m_distances;
};

}  // namespace slam