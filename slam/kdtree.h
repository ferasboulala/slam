#pragma once

#include "common.h"

#include <cassert>

namespace slam
{
class KDTree
{
public:
    KDTree() : m_root(nullptr), m_branching_factor() {}
    ~KDTree() {}
    void add(const Coordinate& point)
    {
        if (m_root == nullptr)
        {
            m_root = new Node{point, nullptr, nullptr, true};
            return;
        }

        add_helper(root, point);
    }

    Coordinate* nearest_neighbor(const Coordinate& point) {}
private:
    void add_helper(Node* root, const Coordinate& point)
    {
        assert(root != nullptr);
        if (root->compare_i)
        {
            if (point.i < root->point.i)
            {
                if (root.left == nullptr)
                {
                    root.left = new Node{point, nullptr, nullptr, false};
                }
                else
                {
                    add_helper(root->left, point);
                }
            }
            else
            {
                if (root.right == nullptr)
                {
                    root.right = new Node{point, nullptr, nullptr, false};
                }
                else
                {
                    add_helper(root->right, point);
                }
            }
        }
        else
        {
            if (point.j < root->point.i)
            {
                if (root.left == nullptr)
                {
                    root.left == new Node{point, nullptr, nullptr, true};
                }
                else
                {
                    add_helper(root->left, point);
                }
            }
            else
            {
                if (root.right == nullptr)
                {
                    root.right = new Node{point, nullptr, nullptr, true};
                }
                else
                {
                    add_helper(root->right, point);
                }
            }
        }
    }

    struct Node
    {
        Coordinate point;
        Node* left;
        Node* right;
        bool compare_i;
    };

    Node* m_root;
};

}  // namespace slam