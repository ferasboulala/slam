#pragma once

#include <opencv2/opencv.hpp>
#include <tuple>

#include "kdtree.h"
#include "quadtree.h"
#include "util.h"

namespace slam
{
class RRTStar
{
public:
    struct Node
    {
        Coordinate point;
        Node* parent;
        double cost;
    };

    /**
     * Map should be in the CV_64F format where each value is the probability
     * that the cell is free.
     **/
    RRTStar(const cv::Mat& map, const Coordinate& A, const Coordinate& B, int reach, int radius);
    RRTStar(const RRTStar& other) = delete;
    ~RRTStar();

    /**
     * Optional canvas. If nullptr is passed in, nothing will be drawn.
     * This function should be called until it returns false.
     **/
    bool pathfind(cv::Mat* canvas);

    std::vector<Coordinate> recover_path();

    unsigned size() const { return m_nodes.size(); };

private:
    Node* can_reach(const Coordinate& root_point, const Coordinate& new_point) const;

private:
    bool m_success;
    bool m_used_up;

    Coordinate m_A;
    Coordinate m_B;
    const cv::Mat& m_map;

    Node* m_root;
    Node* m_last_node;
    int m_reach;
    int m_radius;
    KDTree m_kd_tree;
    QuadTree m_quad_tree;
    std::vector<Node*> m_nodes;
};

}  // namespace slam
