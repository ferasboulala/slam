#pragma once

#include "common.h"

#include <vector>

namespace slam
{
class KDTree
{
private:
    struct Node
    {
        Coordinate point;
        Node* left;
        Node* right;
        bool compare_i;
    };

public:
    KDTree();
    KDTree(const std::vector<Coordinate>& points);

    ~KDTree();
    std::vector<Coordinate> list_points() const;

    void balance();

    void add(const Coordinate& point);

    Coordinate nearest_neighbor(const Coordinate& point) const;

private:
    void list_points_helper(Node* root, std::vector<Coordinate>& points) const;

    void balance(const std::vector<Coordinate>& points);

    void nearest_neighbor_helper(const Coordinate& point, Node* root,
                                 Node** best) const;

    void free(Node* root);

    void add_helper(Node* root, const Coordinate& point);

    Node* m_root;
};

}  // namespace slam