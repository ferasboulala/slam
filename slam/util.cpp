#include "util.h"

#include <cmath>
#include <random>

namespace slam
{

double
pdf_normal_distribution(double stddev, double x)
{
    return 1.0 / stddev / std::sqrt(2 * M_PI) *
           std::exp(-std::pow(x, 2) / 2 / std::pow(stddev, 2));
}

double
pdf_normal_distribution_clamp(double stddev, double x, double multiple_stddev)
{
    if (std::fabs(x) > multiple_stddev * stddev)
        return 0;

    return pdf_normal_distribution(stddev, x);
}

double
pdf_triangular_distribution(double stddev, double x)
{
    const double variance = std::pow(stddev, 2);
    return std::max(0.0,
                    1 / std::sqrt(6 * variance) - std::fabs(x) / 6 / variance);
}

double
sample_normal_distribution(double stddev)
{
    static thread_local std::default_random_engine generator;
    std::normal_distribution<double> distribution(0, stddev);

    return distribution(generator);
}

double
sample_triangular_distribution(double stddev)
{
    static thread_local std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-stddev, stddev);

    return std::sqrt(6) / 2 * (distribution(generator)) +
           distribution(generator);
}

double
normalize_angle(double angle)
{
    angle = std::fmod(angle, 2 * M_PI);
    if (angle > M_PI)
        angle -= 2 * M_PI;

    return angle;
}

} // namespace slam
