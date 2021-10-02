#pragma once

#include "common.h"

#include <functional>

#include <opencv2/opencv.hpp>

namespace slam
{
class AStar
{
public:
    /**
     * Map should be in the CV_64F format where each value is the probability
     * that the cell is free.
     **/
    AStar(const cv::Mat &map, const Coordinate &A, const Coordinate &B);

    ~AStar() = default;

    /**
     * Optional canvas. If nullptr is passed in, nothing will be drawn.
     * This function should be called until it returns false.
     **/
    bool pathfind(cv::Mat *canvas);

    std::vector<Coordinate> recover_path();

private:
    bool m_success = false;
    bool m_used_up = false;

    Coordinate m_A;
    Coordinate m_B;
    const cv::Mat &m_map;

    cv::Mat m_distances;
    std::vector<std::tuple<Coordinate, double>> m_q;

    std::function<double(const Coordinate &X)> m_heuristic;
    std::function<bool(const std::tuple<Coordinate, double> &X,
                       const std::tuple<Coordinate, double> &Y)>
        m_comp;
};

}  // namespace slam