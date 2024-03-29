#include "kdtree.h"

#include <cassert>
#include <vector>

#include "colors.h"
#include "util.h"

namespace slam
{
KDTree::KDTree() : m_root(nullptr) {}
KDTree::KDTree(const std::vector<std::tuple<Coordinate, void*>>& points) : KDTree()
{
    balance(points);
}
KDTree::~KDTree() { free(m_root); }
static void list_points_helper(KDTree::Node* root,
                               std::vector<std::tuple<Coordinate, void*>>& points)
{
    if (root == nullptr)
    {
        return;
    }

    list_points_helper(root->left, points);
    points.push_back(std::tuple<Coordinate, void*>(root->point, root->data));
    list_points_helper(root->right, points);
}

std::vector<std::tuple<Coordinate, void*>> KDTree::list_points() const
{
    std::vector<std::tuple<Coordinate, void*>> points;
    list_points_helper(m_root, points);

    return points;
}

void KDTree::balance()
{
    const std::vector<std::tuple<Coordinate, void*>> points = list_points();
    free(m_root);
    balance(points);
}

static void add_helper(KDTree::Node* root, const Coordinate& point, void* data)
{
    assert(root != nullptr);
    if (root->compare_i)
    {
        if (point.i < root->point.i)
        {
            if (root->left == nullptr)
            {
                root->left = new KDTree::Node{point, nullptr, nullptr, false, data};
            }
            else
            {
                add_helper(root->left, point, data);
            }
        }
        else
        {
            if (root->right == nullptr)
            {
                root->right = new KDTree::Node{point, nullptr, nullptr, false, data};
            }
            else
            {
                add_helper(root->right, point, data);
            }
        }
    }
    else
    {
        if (point.j < root->point.j)
        {
            if (root->left == nullptr)
            {
                root->left = new KDTree::Node{point, nullptr, nullptr, true, data};
            }
            else
            {
                add_helper(root->left, point, data);
            }
        }
        else
        {
            if (root->right == nullptr)
            {
                root->right = new KDTree::Node{point, nullptr, nullptr, true, data};
            }
            else
            {
                add_helper(root->right, point, data);
            }
        }
    }
}

void KDTree::add(const Coordinate& point, void* data)
{
    if (m_root == nullptr)
    {
        m_root = new Node{point, nullptr, nullptr, true, data};
        return;
    }

    add_helper(m_root, point, data);
}

static void nearest_neighbor_helper(const Coordinate& point,
                                    KDTree::Node* root,
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
    else if (euclidean_distance(root->point, point) < euclidean_distance((*best)->point, point))
    {
        *best = root;
    }

    if (root->compare_i)
    {
        if (point.i < root->point.i)
        {
            nearest_neighbor_helper(point, root->left, best);
            if (euclidean_distance((*best)->point, point) > std::abs(point.i - root->point.i))
            {
                nearest_neighbor_helper(point, root->right, best);
            }
        }
        else
        {
            nearest_neighbor_helper(point, root->right, best);
            if (euclidean_distance((*best)->point, point) > std::abs(point.i - root->point.i))
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
            if (euclidean_distance((*best)->point, point) > std::abs(point.j - root->point.j))
            {
                nearest_neighbor_helper(point, root->right, best);
            }
        }
        else
        {
            nearest_neighbor_helper(point, root->right, best);
            if (euclidean_distance((*best)->point, point) > std::abs(point.j - root->point.j))
            {
                nearest_neighbor_helper(point, root->left, best);
            }
        }
    }
}

std::tuple<Coordinate, void*> KDTree::nearest_neighbor(const Coordinate& point) const
{
    if (m_root == nullptr)
    {
        return std::tuple<Coordinate, void*>(point, nullptr);
    }

    Node* best = nullptr;
    nearest_neighbor_helper(point, m_root, &best);

    return std::tuple<Coordinate, void*>(best->point, best->data);
}

void KDTree::balance(const std::vector<std::tuple<Coordinate, void*>>& points)
{
    // FIXME : Implement this with medians
    for (const auto& point : points)
    {
        add(std::get<0>(point), std::get<1>(point));
    }
}

void KDTree::free(Node* root)
{
    if (root == nullptr) return;

    free(root->left);
    free(root->right);

    delete root;
}

static void draw_helper(
    cv::Mat& canvas, KDTree::Node* root, int start_i, int stop_i, int start_j, int stop_j)
{
    if (root == nullptr) return;

    if (root->compare_i)
    {
        cv::line(canvas, {start_j, root->point.i}, {stop_j, root->point.i}, GREEN, 2);
        draw_helper(canvas, root->left, start_i, root->point.i, start_j, stop_j);
        draw_helper(canvas, root->right, root->point.i, stop_i, start_j, stop_j);
    }
    else
    {
        cv::line(canvas, {root->point.j, start_i}, {root->point.j, stop_i}, GREEN, 2);
        draw_helper(canvas, root->left, start_i, stop_i, start_j, root->point.j);
        draw_helper(canvas, root->right, start_i, stop_i, root->point.j, stop_j);
    }
}

void KDTree::draw(cv::Mat& canvas) const
{
    draw_helper(canvas, m_root, 0, canvas.rows, 0, canvas.cols);
}
}  // namespace slam
