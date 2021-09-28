#pragma once

#include "planner.h"

namespace slam
{
class HybridAStar : public Planner
{
public:
    HybridAStar(cv::Mat &mat, const Coordinate &A, const Coordinate &B)
        : Planner(map, A, B)
    {
    }

    bool pathfind(cv::Mat *canvas) { return false; }
    std::vector<Coordinate> recover_path() { return {}; }
};

}  // namespace slam