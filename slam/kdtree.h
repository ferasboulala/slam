#pragma once

#include <tuple>
#include <vector>

#include "util.h"

namespace slam
{
class KDTree
{
public:
    struct Node
    {
        Coordinate point;
        Node* left;
        Node* right;
        bool compare_i;
        void* data;
    };

    KDTree();
    KDTree(const std::vector<std::tuple<Coordinate, void*>>& points);

    ~KDTree();

    std::vector<std::tuple<Coordinate, void*>> list_points() const;

    void balance();

    void add(const Coordinate& point, void* data = nullptr);

    std::tuple<Coordinate, void*> nearest_neighbor(const Coordinate& point) const;

    void draw(cv::Mat& canvas) const;

private:
    void balance(const std::vector<std::tuple<Coordinate, void*>>& points);

    void free(Node* root);

    Node* m_root;
};

}  // namespace slam
