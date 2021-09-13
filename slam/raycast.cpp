#include "raycast.h"
#include "util.h"

#include <cmath>

namespace slam
{
bool
within_boundaries(const Eigen::MatrixXf& map, const int i, const int j)
{
    return i < map.rows() && j < map.cols() && i >= 0 && j >= 0;
}

Pose
raycast(const Eigen::MatrixXf& map, const Pose& pose, double max_distance,
        double step_size)
{
    const double dx = step_size * std::cos(pose.theta);
    const double dy = step_size * std::sin(-pose.theta);

    double x = pose.x;
    double y = pose.y;

    int i = std::round(y);
    int j = std::round(x);

    int prev_i = i;
    int prev_j = j;

    const double max_distance_squared = std::pow(max_distance, 2);

    while (true)
    {
        x += dx;
        y += dy;

        i = std::round(y);
        j = std::round(x);

        if (!within_boundaries(map, i, j))
            return { -1, -1, 0 };

        if (std::pow(x - pose.x, 2) + std::pow(y - pose.y, 2) >
            max_distance_squared)
            return { -1, -1, 0 };

        if ((prev_i != i || prev_j != j) && map(i, j))
            return { x, y, pose.theta };

        prev_i = i;
        prev_j = j;
    }
}

double
measurement_model_beam(double distance, double stddev,
                       const Eigen::MatrixXf& map, const Pose& pose,
                       double max_distance, double step_size)
{
    constexpr double EPSILON = 1e-3;
    const Pose hit = raycast(map, pose, max_distance, step_size);
    if (hit.x == -1)
        return pdf_normal_distribution_clamp(stddev, distance - max_distance) +
               EPSILON;

    const double distance_ =
      std::sqrt(std::pow(hit.x - pose.x, 2) + std::pow(hit.y - pose.y, 2));

    return pdf_normal_distribution_clamp(stddev, distance - distance_) +
           EPSILON;
}

} // namespace slam
