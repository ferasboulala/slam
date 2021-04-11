#pragma once

namespace slam
{

double pdf_normal_distribution(double stddev, double x);
double pdf_triangular_distribution(double stddev, double x);
double sample_normal_distribution(double variance);
double sample_triangular_distribution(double variance);
double normalize_angle(double angle);

} // namespace slam
