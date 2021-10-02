#pragma once

#include "common.h"

#include <opencv2/opencv.hpp>

namespace slam
{
class RRTStar
{
private:
    struct Node
    {
        Coordinate point;
        Node *parent;
        std::vector<Node *> children;
    };

public:
    /**
     * Map should be in the CV_64F format where each value is the probability
     * that the cell is free.
     **/
    RRTStar(const cv::Mat &map, const Coordinate &A, const Coordinate &B,
            double radius);
    ~RRTStar() = default;

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

    Node *m_root;
    double m_radius;
};

}  // namespace slam