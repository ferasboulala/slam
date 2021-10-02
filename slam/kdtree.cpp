#include "kdtree.h"

#include "colors.h"
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
static void list_points_helper(KDTree::Node* root,
                               std::vector<Coordinate>& points)
{
    if (root == nullptr)
    {
        return;
    }

    list_points_helper(root->left, points);
    points.push_back(root->point);
    list_points_helper(root->right, points);
}

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

static void add_helper(KDTree::Node* root, const Coordinate& point)
{
    assert(root != nullptr);
    if (root->compare_i)
    {
        if (point.i < root->point.i)
        {
            if (root->left == nullptr)
            {
                root->left = new KDTree::Node{point, nullptr, nullptr, false};
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
                root->right = new KDTree::Node{point, nullptr, nullptr, false};
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
                root->left = new KDTree::Node{point, nullptr, nullptr, true};
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
                root->right = new KDTree::Node{point, nullptr, nullptr, true};
            }
            else
            {
                add_helper(root->right, point);
            }
        }
    }
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

static void nearest_neighbor_helper(const Coordinate& point, KDTree::Node* root,
                                    KDTree::Node** best)
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

void KDTree::balance(const std::vector<Coordinate>& points)
{
    // Implement this
    for (const Coordinate& point : points)
    {
        add(point);
    }
}

void KDTree::free(Node* root)
{
    if (root == nullptr) return;

    free(root->left);
    free(root->right);

    delete root;
}

static void draw_helper(cv::Mat& canvas, KDTree::Node* root, int start_i,
                        int stop_i, int start_j, int stop_j)
{
    if (root == nullptr) return;

    if (root->compare_i)
    {
        cv::line(canvas, {start_j, root->point.i}, {stop_j, root->point.i},
                 GREY * 255, 2);
        draw_helper(canvas, root->left, start_i, root->point.i, start_j,
                    stop_j);
        draw_helper(canvas, root->right, root->point.i, stop_i, start_j,
                    stop_j);
    }
    else
    {
        cv::line(canvas, {root->point.j, start_i}, {root->point.j, stop_i},
                 GREY * 255, 2);
        draw_helper(canvas, root->left, start_i, stop_i, start_j,
                    root->point.j);
        draw_helper(canvas, root->right, start_i, stop_i, root->point.j,
                    stop_j);
    }
}

void KDTree::draw(cv::Mat& canvas) const
{
    draw_helper(canvas, m_root, 0, canvas.rows, 0, canvas.cols);
}

}  // namespace sla