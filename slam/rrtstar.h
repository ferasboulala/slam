#pragma once

#include "kdtree.h"
#include "util.h"

#include <opencv2/opencv.hpp>

#include <tuple>

namespace slam
{
class RRTStar
{
public:
    struct Node
    {
        Coordinate point;
        Node *parent;
        std::vector<Node *> children;
    };

    /**
     * Map should be in the CV_64F format where each value is the probability
     * that the cell is free.
     **/
    RRTStar(const cv::Mat &map, const Coordinate &A, const Coordinate &B,
            double radius);
    ~RRTStar();

    /**
     * Optional canvas. If nullptr is passed in, nothing will be drawn.
     * This function should be called until it returns false.
     **/
    bool pathfind(cv::Mat *canvas);

    std::vector<Coordinate> recover_path();

    unsigned size() const { return m_size; };
private:
    bool m_success;
    bool m_used_up;

    Coordinate m_A;
    Coordinate m_B;
    const cv::Mat &m_map;

    Node *m_root;
    Node *m_last_node;
    unsigned m_size;
    double m_radius;
    KDTree m_tree;
};

}  // namespace slam
