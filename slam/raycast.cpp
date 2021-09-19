#include "raycast.h"

namespace slam
{
template <>
Pose raycast<double>(const cv::Mat& map, const Pose& pose, double max_distance,
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

    const double max_distance_squared = std::pow(max_distance, 2);

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

        const double diffx = x - pose.x;
        const double diffy = y - pose.y;

        if (diffx * diffx + diffy * diffy >= max_distance_squared)
            return {-1, -1, pose.theta};

        if (map.at<double>(i, j) < 0.5) // probability that it is free
        {
            return {x, y, pose.theta};
        }

        prev_i = i;
        prev_j = j;
    }
}

template <>
Pose raycast<int>(const cv::Mat& map, const Pose& pose, double max_distance,
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

        if (prev_i == i && prev_j == j)
        {
            continue;
        }

        if (!within_boundaries(map, i, j)) return {-1, -1, pose.theta};

        const double diffx = x - pose.x;
        const double diffy = y - pose.y;
        if (diffx * diffx + diffy * diffy >= max_distance_squared)
            return {-1, -1, pose.theta};

        if (!map.at<int>(i, j)) return {x, y, pose.theta};

        prev_i = i;
        prev_j = j;
    }
}

void raycast_mapping(Particle& particle, double z, double z_max,
                     double step_size)
{
    const double dx = step_size * std::cos(particle.pose.theta);
    const double dy = step_size * std::sin(particle.pose.theta);

    double x = particle.pose.x;
    double y = particle.pose.y;

    auto coord = pose_to_image_coordinates(particle.map, {x, y, 0});
    int i = std::get<0>(coord);
    int j = std::get<1>(coord);

    int prev_i = i;
    int prev_j = j;

    const double z_squared = z * z;
    const double z_max_squared = z_max * z_max;

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
        constexpr double Locc = 0.2;
        constexpr double Lfree = 0.8;

        const double diffx = x - particle.pose.x;
        const double diffy = y - particle.pose.y;
        const double distance_squared = diffx * diffx + diffy * diffy;

        if (distance_squared < z_squared)
        {
            particle.map.at<double>(i, j) *= Lfree / L0;
            particle.map.at<double>(i, j) =
                std::min(1.0, particle.map.at<double>(i, j));
        }
        else
        {
            if (z_squared != z_max_squared)
                particle.map.at<double>(i, j) *= Locc / L0;
            break;
        }

        prev_i = i;
        prev_j = j;
    }
}

double measurement_model_beam(double distance, double stddev,
                              Particle& particle, double max_distance,
                              double step_size)
{
    const Pose hit = raycast<double>(particle.map, particle.pose, max_distance);
    raycast_mapping(particle, distance, max_distance, step_size);

    constexpr double EPSILON = 0.1;
    if (hit.x == -1)  // no hit
        return pdf_normal_distribution_clamp(stddev, distance - max_distance) +
               EPSILON;

    const double distance_ = std::sqrt(std::pow(hit.x - particle.pose.x, 2) +
                                       std::pow(hit.y - particle.pose.y, 2));

    return pdf_normal_distribution_clamp(stddev, distance - distance_) +
           EPSILON;
}

}  // namespace slam