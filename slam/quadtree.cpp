#include "quadtree.h"

#include <array>
#include <cassert>

#include "colors.h"

namespace slam
{
QuadTree::QuadTree() : m_root(nullptr) {}
void QuadTree::free(Node* root)
{
    if (root == nullptr) return;

    free(root->NE);
    free(root->NW);
    free(root->SE);
    free(root->SW);

    delete root;
}

QuadTree::~QuadTree() { free(m_root); }
inline std::array<std::pair<Coordinate, Coordinate>, 4> QuadTree::regions(const Coordinate& point,
                                                                          const Coordinate& start,
                                                                          const Coordinate& stop)
{
    std::array<std::pair<Coordinate, Coordinate>, 4> subregions = {
        std::pair<Coordinate, Coordinate>({start.i, point.j}, {point.i, stop.j}),
        {start, point},
        {point, stop},
        {{point.i, start.j}, {stop.i, point.j}}};

    return subregions;
}

void QuadTree::add_helper(Node* root, const Coordinate& point, void* data)
{
    assert(root != nullptr);
    if (point.i >= root->point.i && point.j < root->point.j)
    {
        if (root->SW == nullptr)
        {
            root->SW = new Node{point, data, nullptr, nullptr, nullptr, nullptr};
            return;
        }
        add_helper(root->SW, point, data);
    }
    else if (point.i < root->point.i && point.j < root->point.j)
    {
        if (root->NW == nullptr)
        {
            root->NW = new Node{point, data, nullptr, nullptr, nullptr, nullptr};
            return;
        }
        add_helper(root->NW, point, data);
    }
    else if (point.i >= root->point.i && point.j >= root->point.j)
    {
        if (root->SE == nullptr)
        {
            root->SE = new Node{point, data, nullptr, nullptr, nullptr, nullptr};
            return;
        }
        add_helper(root->SE, point, data);
    }
    else
    {
        if (root->NE == nullptr)
        {
            root->NE = new Node{point, data, nullptr, nullptr, nullptr, nullptr};
            return;
        }
        add_helper(root->NE, point, data);
    }
}

void QuadTree::add(const Coordinate& point, void* data)
{
    if (m_root == nullptr)
    {
        m_root = new Node{point, data, nullptr, nullptr, nullptr, nullptr};
        return;
    }

    add_helper(m_root, point, data);
}

void QuadTree::range_query_helper(const Node* root,
                                  const Coordinate& start,
                                  const Coordinate& stop,
                                  std::vector<std::tuple<Coordinate, void*>>& result,
                                  const Coordinate& node_start,
                                  const Coordinate& node_stop)
{
    if (root == nullptr) return;

    if (within_bounding_box(root->point, start, stop))
    {
        result.push_back(std::tuple<Coordinate, void*>(root->point, root->data));
    }

    const auto subregions = regions(root->point, node_start, node_stop);
    if (bounding_boxes_intersect(subregions[0].first, subregions[0].second, start, stop))
    {
        range_query_helper(
            root->NE, start, stop, result, subregions[0].first, subregions[0].second);
    }
    if (bounding_boxes_intersect(subregions[1].first, subregions[1].second, start, stop))
    {
        range_query_helper(
            root->NW, start, stop, result, subregions[1].first, subregions[1].second);
    }
    if (bounding_boxes_intersect(subregions[2].first, subregions[2].second, start, stop))
    {
        range_query_helper(
            root->SE, start, stop, result, subregions[2].first, subregions[2].second);
    }
    if (bounding_boxes_intersect(subregions[3].first, subregions[3].second, start, stop))
    {
        range_query_helper(
            root->SW, start, stop, result, subregions[3].first, subregions[3].second);
    }
}

std::vector<std::tuple<Coordinate, void*>> QuadTree::range_query(
    const Coordinate& upper_left, const Coordinate& bottom_right) const
{
    std::vector<std::tuple<Coordinate, void*>> result;

    range_query_helper(m_root,
                       upper_left,
                       bottom_right,
                       result,
                       {0, 0},
                       {std::numeric_limits<int>::max(), std::numeric_limits<int>::max()});

    return result;
}

void QuadTree::draw_helper(cv::Mat& canvas,
                           const Node* root,
                           const Coordinate& start,
                           const Coordinate& stop)
{
    if (root == nullptr) return;

    cv::line(canvas, {root->point.j, start.i}, {root->point.j, stop.i}, GREEN);
    cv::line(canvas, {start.j, root->point.i}, {stop.j, root->point.i}, GREEN);

    const auto subregions = regions(root->point, start, stop);
    draw_helper(canvas, root->NE, subregions[0].first, subregions[0].second);
    draw_helper(canvas, root->NW, subregions[1].first, subregions[1].second);
    draw_helper(canvas, root->SE, subregions[2].first, subregions[2].second);
    draw_helper(canvas, root->SW, subregions[3].first, subregions[3].second);
}

void QuadTree::draw(cv::Mat& canvas) const
{
    draw_helper(canvas, m_root, {0, 0}, {canvas.rows - 1, canvas.cols - 1});
}
}  // namespace slam
