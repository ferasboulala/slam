#pragma once

#include <tuple>
#include <vector>

#include "util.h"

namespace slam
{
class QuadTree
{
public:
    QuadTree();
    ~QuadTree();

    void add(const Coordinate& point, void* data = nullptr);

    std::vector<std::tuple<Coordinate, void*>> range_query(const Coordinate& upper_left,
                                                           const Coordinate& bottom_right) const;

    void draw(cv::Mat& canvas) const;

private:
    struct Node
    {
        Coordinate point;
        void* data;
        Node* NE;
        Node* NW;
        Node* SE;
        Node* SW;
    };

    static void free(Node* root);

    static void draw_helper(cv::Mat& canvas,
                            const Node* root,
                            const Coordinate& start,
                            const Coordinate& stop);

    static void add_helper(Node* root, const Coordinate& point, void* data);
    static inline std::array<std::pair<Coordinate, Coordinate>, 4> regions(const Coordinate& point,
                                                                           const Coordinate& start,
                                                                           const Coordinate& stop);
    static void range_query_helper(const Node* root,
                                   const Coordinate& start,
                                   const Coordinate& stop,
                                   std::vector<std::tuple<Coordinate, void*>>& result,
                                   const Coordinate& node_start,
                                   const Coordinate& node_stop);

private:
    Node* m_root;
};

}  // namespace slam