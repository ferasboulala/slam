#pragma once

namespace slam
{

struct Pose
{
    double x;
    double y;
    double theta;
};

inline bool
operator==(const Pose& lhs, const Pose& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.theta == rhs.theta;
}
}
