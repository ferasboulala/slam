#pragma once

#include "common.h"

#include <cassert>

namespace slam
{
class KDTree
{
public:
    KDTree() : m_root(nullptr), m_branching_factor() {}
    ~KDTree() { free(m_root); }
    void add(const Coordinate& point)
    {
        if (m_root == nullptr)
        {
            m_root = new Node{point, nullptr, nullptr, true};
            return;
        }

        add_helper(root, point);
    }

    Coordinate nearest_neighbor(const Coordinate& point) const
    {
        if (m_root == nullptr)
        {
            return point;
        }

        Node* best = nullptr;
        nearest_neighbor_helper(m_root, point, &best);

        return best->point;
    }

private:
    void nearest_neighbor_helper(const Coordinate& point, const Node* root,
                                 Node** best) const
    {
        if (root == nullptr)
        {
            return;
        }

        if (best == nullptr)
        {
            best = root;
        }
        else if (euclidean_distance(root->point, point) <
                 euclidean_distance((*best)->point, point))
        {
            *best = root;
        }

        if (root->compare_i)
        {
            if (point.i < root->point.i)
            {
                nearest_neighbor_helper(point, root->left, best);
                if (euclidean_distance(root->point, point) >
                    std::abs(point.i - root.point.i))
                {
                    nearest_neighbor_helper(point, root->right, best);
                }
            }
            else
            {
                nearest_neighbor_helper(point, root->right, best);
                if (euclidean_distance(root->point, point) >
                    std::abs(point.i - root.point.i))
                {
                    nearest_neighbor_helper(point, root->left, best);
                }
            }
        }
        else
        {
            if (point.j < root->point.j)
            {
                nearest_neighbor_helper(point, root->left, best);
                if (euclidean_distance(root->point, point) >
                    std::abs(point.j - root.point.j))
                {
                    nearest_neighbor_helper(point, root->right, best);
                }
            }
            else
            {
                nearest_neighbor_helper(point, root->right, best);
                if (euclidean_distance(root->point, point) >
                    std::abs(point.j - root.point.j))
                {
                    nearest_neighbor_helper(point, root->left, best);
                }
            }
        }
    }

    void free(Node* root)
    {
        if (root == nullptr) return;

        free(root->left);
        free(root->right);

        delete root;
    }

    void add_helper(Node* root, const Coordinate& point)
    {
        assert(root != nullptr);
        if (root->compare_i)
        {
            if (point.i < root->point.i)
            {
                if (root->left == nullptr)
                {
                    root->left = new Node{point, nullptr, nullptr, false};
                }
                else
                {
                    add_helper(root->left, point);
                }
            }
            else
            {
                if (root->right == nullptr)
                {
                    root->right = new Node{point, nullptr, nullptr, false};
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
                if (root->left == nullptr)
                {
                    root->left == new Node{point, nullptr, nullptr, true};
                }
                else
                {
                    add_helper(root->left, point);
                }
            }
            else
            {
                if (root->right == nullptr)
                {
                    root->right = new Node{point, nullptr, nullptr, true};
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