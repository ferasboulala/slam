#include "kdtree.h"

#include "common.h"

#include <cassert>
#include <vector>

namespace slam
{
KDTree::KDTree() : m_root(nullptr) {}
KDTree::KDTree(const std::vector<Coordinate>& points) : KDTree()
{
    balance(points);
}

KDTree::~KDTree() { free(m_root); }
std::vector<Coordinate> KDTree::list_points() const
{
    std::vector<Coordinate> points;
    list_points_helper(m_root, points);

    return points;
}

void KDTree::balance()
{
    const std::vector<Coordinate> points = list_points();
    free(m_root);
    balance(points);
}

void KDTree::add(const Coordinate& point)
{
    if (m_root == nullptr)
    {
        m_root = new Node{point, nullptr, nullptr, true};
        return;
    }

    add_helper(m_root, point);
}

Coordinate KDTree::nearest_neighbor(const Coordinate& point) const
{
    if (m_root == nullptr)
    {
        return point;
    }

    Node* best = nullptr;
    nearest_neighbor_helper(point, m_root, &best);

    return best->point;
}

void KDTree::list_points_helper(Node* root,
                                std::vector<Coordinate>& points) const
{
    if (root == nullptr)
    {
        return;
    }

    list_points_helper(root->left, points);
    points.push_back(root->point);
    list_points_helper(root->right, points);
}

void KDTree::balance(const std::vector<Coordinate>& points)
{
    // Implement this
    for (const Coordinate& point : points)
    {
        add(point);
    }
}

void KDTree::nearest_neighbor_helper(const Coordinate& point, Node* root,
                                     Node** best) const
{
    if (root == nullptr)
    {
        return;
    }

    if (*best == nullptr)
    {
        *best = root;
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
            if (euclidean_distance((*best)->point, point) >
                std::abs(point.i - root->point.i))
            {
                nearest_neighbor_helper(point, root->right, best);
            }
        }
        else
        {
            nearest_neighbor_helper(point, root->right, best);
            if (euclidean_distance((*best)->point, point) >
                std::abs(point.i - root->point.i))
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
            if (euclidean_distance((*best)->point, point) >
                std::abs(point.j - root->point.j))
            {
                nearest_neighbor_helper(point, root->right, best);
            }
        }
        else
        {
            nearest_neighbor_helper(point, root->right, best);
            if (euclidean_distance((*best)->point, point) >
                std::abs(point.j - root->point.j))
            {
                nearest_neighbor_helper(point, root->left, best);
            }
        }
    }
}

void KDTree::free(Node* root)
{
    if (root == nullptr) return;

    free(root->left);
    free(root->right);

    delete root;
}

void KDTree::add_helper(Node* root, const Coordinate& point)
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
        if (point.j < root->point.j)
        {
            if (root->left == nullptr)
            {
                root->left = new Node{point, nullptr, nullptr, true};
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

}  // namespace sla