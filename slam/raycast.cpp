#include "raycast.h"
#include "util.h"

#include <algorithm>
#include <cmath>

namespace slam
{

Pose raycast(const Eigen::MatrixXf& map, const Pose& pose, double max_distance,
             double step_size)
{
    const double dx = step_size * std::cos(pose.theta);
    const double dy = step_size * std::sin(pose.theta);

    double x = pose.x;
    double y = pose.y;

    auto coord = pose_to_image_coordinates(map, {x, y, 0});
    int i = std::get<0>(coord);
    int j = std::get<1>(coord);

    int prev_i = i;
    int prev_j = j;

    const double max_distance_squared = std::pow(max_distance, 2);

    while (true)
    {
        x += dx;
        y += dy;

        coord = pose_to_image_coordinates(map, {x, y, 0});
        i = std::get<0>(coord);
        j = std::get<1>(coord);

        if (!within_boundaries(map, i, j)) return {-1, -1, pose.theta};

        if (std::pow(x - pose.x, 2) + std::pow(y - pose.y, 2) >
            max_distance_squared)
            return {-1, -1, pose.theta};

        if ((prev_i != i || prev_j != j) && map(i, j))
            return {x, y, pose.theta};

        prev_i = i;
        prev_j = j;
    }
}

double measurement_model_beam(double distance, double stddev,
                              const Eigen::MatrixXf& map, const Pose& pose,
                              double max_distance, double step_size)
{
    const Pose hit = raycast(map, pose, max_distance, step_size);

    constexpr double EPSILON = 0.01;
    if (hit.x == -1)  // no hit
        return pdf_normal_distribution_clamp(stddev, distance - max_distance) +
               EPSILON;

    const double distance_ =
        std::sqrt(std::pow(hit.x - pose.x, 2) + std::pow(hit.y - pose.y, 2));

    return pdf_normal_distribution_clamp(stddev, distance - distance_) +
           EPSILON;
}

double measurement_model_beam_score(double distance, double stddev,
                              const Eigen::MatrixXf& map, const Pose& pose,
                              double max_distance, double step_size)
{
    const Pose hit = raycast(map, pose, max_distance, step_size);
    const double distance_ = 
        hit.x == -1 ? max_distance : std::sqrt(std::pow(hit.x - pose.x, 2) + std::pow(hit.y - pose.y, 2));
    
    return std::max(0.0, 1 - std::fabs(distance - distance_) / stddev);
}

}  // namespace slam