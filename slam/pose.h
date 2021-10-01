#pragma once

#include <opencv2/opencv.hpp>

namespace slam
{
struct Pose
{
    double x;
    double y;
    double theta;
};

inline bool operator==(const Pose &lhs, const Pose &rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.theta == rhs.theta;
}

struct Odometry
{
    double rotation_1;
    double translation;
    double rotation_2;
};

struct Velocity
{
    double v;
    double w;
};

struct Particle
{
    Pose pose;
    double weight;
    cv::Mat map;
};

struct Box
{
    int start_i;
    int start_j;
    int stop_i;
    int stop_j;
};

}  // namespace slam
