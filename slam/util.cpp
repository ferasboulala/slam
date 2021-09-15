#include "util.h"

#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <random>

#include <thirdparty/log.h>

namespace slam
{
double pdf_normal_distribution(double stddev, double x)
{
    return (1.0 / (stddev * std::sqrt(2 * M_PI))) *
           std::exp(-0.5 * std::pow(x / stddev, 2));
}

double pdf_normal_distribution_clamp(double stddev, double x,
                                     double multiple_stddev)
{
    if (std::fabs(x) > multiple_stddev * stddev) return 0;

    return pdf_normal_distribution(stddev, x);
}

double pdf_triangular_distribution(double stddev, double x)
{
    const double variance = std::pow(stddev, 2);
    return std::max(0.0,
                    1 / std::sqrt(6 * variance) - std::fabs(x) / 6 / variance);
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

    return std::sqrt(6) / 2 * (distribution(generator)) +
           distribution(generator);
}

double normalize_angle(double angle)
{
    angle = std::fmod(angle, 2 * M_PI);
    if (angle > M_PI) angle -= 2 * M_PI;

    return angle;
}

std::tuple<int, int> pose_to_image_coordinates(const Eigen::MatrixXf& map,
                                               const Pose& pose)
{
    return std::tuple<int, int>(map.rows() - pose.y - 1, pose.x);
}

bool within_boundaries(const Eigen::MatrixXf& map, const int i, const int j)
{
    return i < map.rows() && j < map.cols() && i >= 0 && j >= 0;
}

}  // namespace slam
