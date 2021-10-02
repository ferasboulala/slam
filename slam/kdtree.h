#pragma once

#include "common.h"

#include <vector>

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
    };

    KDTree();
    KDTree(const std::vector<Coordinate>& points);

    ~KDTree();

    std::vector<Coordinate> list_points() const;

    void balance();

    void add(const Coordinate& point);

    Coordinate nearest_neighbor(const Coordinate& point) const;

    void draw(cv::Mat& canvas) const;

private:
    void balance(const std::vector<Coordinate>& points);

    void free(Node* root);

    Node* m_root;
};

}  // namespace slam