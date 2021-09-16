#include "raycast.h"
#include "util.h"

#include <algorithm>
#include <cmath>

namespace slam
{

Pose raycast(const cv::Mat& map, const Pose& pose, double max_distance,
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
        std::tie(i, j) = coord;

        if (prev_i == i && prev_j == j) {
            continue;
        }

        if (!within_boundaries(map, i, j)) return {x, y, pose.theta};

        if (std::pow(x - pose.x, 2) + std::pow(y - pose.y, 2) >
            max_distance_squared)
            return {-1, -1, pose.theta};

        if ((prev_i != i || prev_j != j) && map.at<int>(i, j))
            return {x, y, pose.theta};

        prev_i = i;
        prev_j = j;
    }
}

Pose raycast_mapping(Particle &particle, cv::Mat& occupied, cv::Mat& free,
             Box &box, double distance, double max_distance, double stddev, double step_size)
{
}

double measurement_model_beam(double distance, double stddev,
                              Particle &particle, cv::Mat& occupied, cv::Mat& free,
                              Box &box, double max_distance, double step_size)
{
    const Pose hit = raycast_mapping(particle, occupied, free, box, distance, max_distance, stddev, step_size);

    constexpr double EPSILON = 1e-2;
    if (hit.x == -1)  // no hit
        return pdf_normal_distribution_clamp(stddev, distance - max_distance) +
               EPSILON;

    const double distance_ =
        std::sqrt(std::pow(hit.x - particle.pose.x, 2) + std::pow(hit.y - particle.pose.y, 2));

    return pdf_normal_distribution_clamp(stddev, distance - distance_) +
           EPSILON;
}

}  // namespace slam