#pragma once

#include "common.h"

#include <vector>

#include <opencv2/opencv.hpp>

namespace slam
{
class Planner
{
public:
    Planner(const cv::Mat &map, const Coordinate &A, const Coordinate &B)
        : m_A(A), m_B(B), m_map(map)
    {
    }

    ~Planner() = default;

    virtual bool pathfind(cv::Mat *canvas) = 0;
    virtual std::vector<Coordinate> recover_path() = 0;

    Coordinate m_A;
    Coordinate m_B;
    const cv::Mat &m_map;
};

}  // namespace slam