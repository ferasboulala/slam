#include "raycast.h"

#include <cassert>

namespace slam
{
template <>
Pose raycast<double>(const cv::Mat& map,
                     const Pose& pose,
                     double max_distance_squared,
                     double step_size)
{
    const double dx = step_size * std::cos(pose.theta);
    const double dy = step_size * std::sin(pose.theta);

    double x = pose.x;
    double y = pose.y;

    int i, j;
    auto coord = pose_to_image_coordinates(map, {x, y, 0});
    std::tie(i, j) = coord;

    int prev_i = i;
    int prev_j = j;

    while (true)
    {
        x += dx;
        y += dy;

        coord = pose_to_image_coordinates(map, {x, y, 0});
        std::tie(i, j) = coord;

        if (prev_i == i && prev_j == j)
        {
            continue;
        }

        const double d = euclidean_distance_squared(x, y, pose.x, pose.y);
        if (d >= max_distance_squared) return {-1, -1, pose.theta};

        if (!within_boundaries(map, i, j)) return {-1, -1, pose.theta};

        if (map.at<double>(i, j) < 0.5)  // probability that it is free
        {
            return {x, y, pose.theta};
        }

        prev_i = i;
        prev_j = j;
    }
}

template <>
Pose raycast<int>(const cv::Mat& map,
                  const Pose& pose,
                  double max_distance_squared,
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

    while (true)
    {
        x += dx;
        y += dy;

        coord = pose_to_image_coordinates(map, {x, y, 0});
        std::tie(i, j) = coord;

        if (prev_i == i && prev_j == j)
        {
            continue;
        }

        if (!within_boundaries(map, i, j)) return {-1, -1, pose.theta};

        const double d = euclidean_distance_squared(x, y, pose.x, pose.y);
        if (d >= max_distance_squared) return {-1, -1, pose.theta};

        if (!map.at<int>(i, j)) return {x, y, pose.theta};

        prev_i = i;
        prev_j = j;
    }
}

void raycast_mapping(Particle& particle, double z_squared, double z_max_squared, double step_size)
{
    assert(z_squared <= z_max_squared);

    const double dx = step_size * std::cos(particle.pose.theta);
    const double dy = step_size * std::sin(particle.pose.theta);

    double x = particle.pose.x;
    double y = particle.pose.y;

    auto coord = pose_to_image_coordinates(particle.map, {x, y, 0});
    int i = std::get<0>(coord);
    int j = std::get<1>(coord);

    int prev_i = i;
    int prev_j = j;

    while (true)
    {
        x += dx;
        y += dy;

        coord = pose_to_image_coordinates(particle.map, {x, y, 0});
        std::tie(i, j) = coord;

        if (prev_i == i && prev_j == j)
        {
            continue;
        }

        if (!within_boundaries(particle.map, i, j)) return;

        constexpr double L0 = 0.5;
        constexpr double Locc = 0.3;
        constexpr double Lfree = 0.7;

        const double distance_squared =
            euclidean_distance_squared(x, y, particle.pose.x, particle.pose.y);
        if (distance_squared < z_squared)
        {
            particle.map.at<double>(i, j) *= Lfree / L0;
            particle.map.at<double>(i, j) = std::min(1.0, particle.map.at<double>(i, j));
        }
        else
        {
            if (z_squared != z_max_squared) particle.map.at<double>(i, j) *= Locc / L0;
            break;
        }

        prev_i = i;
        prev_j = j;
    }
}

double measurement_model_beam(double distance_squared,
                              double stddev,
                              Particle& particle,
                              double max_distance_squared,
                              double step_size)
{
    const Pose hit = raycast<double>(particle.map, particle.pose, max_distance_squared);
    raycast_mapping(particle, distance_squared, max_distance_squared, step_size);

    constexpr double EPSILON = 0.1;
    if (hit.x == -1)  // no hit
        return pdf_normal_distribution_clamp(
                   stddev, std::sqrt(distance_squared) - std::sqrt(max_distance_squared)) +
               EPSILON;

    const double distance = euclidean_distance(hit.x, hit.y, particle.pose.x, particle.pose.y);

    return pdf_normal_distribution_clamp(stddev, distance - std::sqrt(distance_squared)) + EPSILON;
}

}  // namespace slam