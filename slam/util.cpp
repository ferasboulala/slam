#include "util.h"

#include <chrono>
#include <cmath>
#include <random>

namespace slam
{
double pdf_normal_distribution(double stddev, double x)
{
    return (1.0 / (stddev * std::sqrt(2 * M_PI))) * std::exp(-0.5 * std::pow(x / stddev, 2));
}

double pdf_normal_distribution_clamp(double stddev, double x, double multiple_stddev)
{
    if (std::fabs(x) > multiple_stddev * stddev) return 0;

    return pdf_normal_distribution(stddev, x);
}

double pdf_triangular_distribution(double stddev, double x)
{
    const double variance = std::pow(stddev, 2);
    return std::max(0.0, 1 / std::sqrt(6 * variance) - std::fabs(x) / 6 / variance);
}

double sample_normal_distribution(double stddev)
{
    static thread_local std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<double> distribution(0, stddev);

    return distribution(generator);
}

double sample_triangular_distribution(double stddev)
{
    static thread_local std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> distribution(-stddev, stddev);

    return std::sqrt(6) / 2 * (distribution(generator)) + distribution(generator);
}

double normalize_angle(double angle)
{
    angle = std::fmod(angle, 2 * M_PI);
    if (angle > M_PI) angle -= 2 * M_PI;

    return angle;
}

Coordinate random_point(const cv::Mat& map)
{
    static thread_local std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<std::mt19937::result_type> distribution_i(0, map.rows);
    std::uniform_int_distribution<std::mt19937::result_type> distribution_j(0, map.cols);

    return {static_cast<int>(distribution_i(generator)), static_cast<int>(distribution_j(generator))};
}

Pose average_pose(const std::vector<Particle>& particles)
{
    double avg_x = 0;
    double avg_y = 0;
    double theta_x = 0;
    double theta_y = 0;
    for (const Particle& particle : particles)
    {
        avg_x += particle.pose.x;
        avg_y += particle.pose.y;
        theta_x += std::cos(particle.pose.theta);
        theta_y += std::sin(particle.pose.theta);
    }

    avg_x /= particles.size();
    avg_y /= particles.size();
    const double avg_theta = std::atan2(theta_y, theta_x);

    return {avg_x, avg_y, avg_theta};
}

}  // namespace slam
