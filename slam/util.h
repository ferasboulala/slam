#pragma once

#include <array>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

#include "pose.h"

namespace slam
{
double pdf_normal_distribution(double stddev, double x);
double pdf_normal_distribution_clamp(double stddev, double x, double multiple_stddev = 4);
double pdf_triangular_distribution(double stddev, double x);
double sample_normal_distribution(double stddev);
double sample_triangular_distribution(double stddev);
double normalize_angle(double angle);
Pose average_pose(const std::vector<Particle>& particles);

struct Coordinate
{
    int i;
    int j;

    inline bool operator==(const Coordinate& other) const { return i == other.i && j == other.j; }
    inline bool operator!=(const Coordinate& other) const { return !(*this == other); }
};

inline std::tuple<int, int> pose_to_image_coordinates(const cv::Mat& map, const Pose& pose)
{
    return std::tuple<int, int>(map.rows - pose.y - 1, pose.x);
}

inline Coordinate pose_to_image_coordinates_(const cv::Mat& map, const Pose& pose)
{
    return Coordinate{static_cast<int>(map.rows - pose.y - 1), static_cast<int>(pose.x)};
}

inline Pose image_coordinates_to_pose(const cv::Mat& map, const Coordinate& coord)
{
    return Pose{static_cast<double>(coord.j), static_cast<double>(map.rows - coord.i), 0};
}

inline bool within_boundaries(const cv::Mat& map, const int i, const int j)
{
    return i < map.rows && j < map.cols && i >= 0 && j >= 0;
}

inline bool within_boundaries(const cv::Mat& map, Coordinate& point)
{
    return point.i < map.rows && point.j < map.cols && point.i >= 0 && point.j >= 0;
}

inline bool within_bounding_box(const Coordinate& point, const Coordinate& start, const Coordinate& stop)
{
    return point.i >= start.i && point.i <= stop.i && point.j >= start.j && point.j <= stop.j;
}

inline bool bounding_boxes_intersect(const Coordinate& a_start,
                                     const Coordinate& a_stop,
                                     const Coordinate& b_start,
                                     const Coordinate& b_stop)
{
    return !(a_stop.i < b_start.i || a_start.i > b_stop.i || a_stop.j < b_start.j || a_start.j > b_stop.j);
}

inline double log_odds(double p) { return std::log(p / (1 - p)); }
inline double log_odds_inv(double l) { return 1 - (1.0 / (1 + std::exp(l))); }
Coordinate random_point(const cv::Mat& map);

inline std::array<Coordinate, 8> adjacency_8(const Coordinate& X)
{
    const std::array<Coordinate, 8> neighbourhood = {Coordinate{X.i + 1, X.j + 1},
                                                     Coordinate{X.i + 1, X.j - 1},
                                                     Coordinate{X.i - 1, X.j + 1},
                                                     Coordinate{X.i - 1, X.j - 1},
                                                     Coordinate{X.i + 1, X.j},
                                                     Coordinate{X.i, X.j + 1},
                                                     Coordinate{X.i - 1, X.j},
                                                     Coordinate{X.i, X.j - 1}};

    return neighbourhood;
}

inline std::array<Coordinate, 4> adjacency_4(const Coordinate& X)
{
    const std::array<Coordinate, 4> neighbourhood = {
        Coordinate{X.i + 1, X.j}, Coordinate{X.i, X.j + 1}, Coordinate{X.i - 1, X.j}, Coordinate{X.i, X.j - 1}};

    return neighbourhood;
}

namespace
{
inline bool within_boundaries(const cv::Mat& map, const Coordinate& X)
{
    return X.i < map.rows && X.j < map.cols && X.i >= 0 && X.j >= 0;
}

inline int manhattan_distance(const Coordinate& A, const Coordinate& B)
{
    return std::abs(A.i - B.i) + std::abs(A.j - B.j);
}

inline int euclidean_distance_squared(const Coordinate& A, const Coordinate& B)
{
    const int idiff = A.i - B.i;
    const int jdiff = A.j - B.j;

    return idiff * idiff + jdiff * jdiff;
}

inline double euclidean_distance(const Coordinate& A, const Coordinate& B)
{
    return std::sqrt(static_cast<double>(euclidean_distance_squared(A, B)));
}

inline double euclidean_distance(const Pose& A, const Pose& B)
{
    const double dx = B.x - A.x;
    const double dy = B.y - A.y;

    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

}  // namespace slam
