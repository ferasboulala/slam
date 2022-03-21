#include "rrtstar.h"

#include <cmath>
#include <limits>
#include <optional>

#include "colors.h"
#include "raycast.h"
#include "thirdparty/log.h"

static constexpr unsigned MAX_N_NODES = 1e9 / sizeof(slam::RRTStar::Node);  // 1 megabyte

namespace slam
{
RRTStar::RRTStar(
    const cv::Mat &map, const Coordinate &A, const Coordinate &B, int reach, int radius, int seed)
    : m_A(A),
      m_B(B),
      m_map(map),
      m_root(new Node{A, nullptr, 0}),
      m_last_node(nullptr),
      m_reach(reach),
      m_radius(radius),
      m_seed(seed)
{
    assert(radius >= reach);
    m_kd_tree.add(A, reinterpret_cast<void *>(m_root));
    m_quad_tree.add(A, reinterpret_cast<void *>(m_root));
}

RRTStar::~RRTStar()
{
    for (Node *node : m_nodes) delete node;

    delete m_root;
}

RRTStar::Node *RRTStar::can_reach(const Coordinate &root_point, const Coordinate &new_point) const
{
    double dist = euclidean_distance(root_point, new_point);
    if (!dist)
    {
        return nullptr;
    }
    Coordinate new_point_to_add;
    const double di = (new_point.i - root_point.i) / dist;
    const double dj = (new_point.j - root_point.j) / dist;
    new_point_to_add = Coordinate{static_cast<int>(root_point.i + di * m_reach),
                                  static_cast<int>(root_point.j + dj * m_reach)};

    if (!within_boundaries(m_map, new_point_to_add.i, new_point_to_add.j)) return nullptr;

    if (m_map.at<double>(new_point_to_add.i, new_point_to_add.j) < 0.5) return nullptr;

    Pose pose = image_coordinates_to_pose(m_map, root_point);
    const double dx = dj;
    const double dy = -di;
    const double angle = std::atan2(dy, dx);
    pose.theta = angle;

    if (raycast<double>(m_map, pose, m_reach * m_reach).x != -1) return nullptr;

    return new Node{new_point_to_add, nullptr, std::numeric_limits<double>::max()};
}

bool RRTStar::pathfind(cv::Mat *canvas)
{
    while (m_nodes.size() < MAX_N_NODES)
    {
        const Coordinate new_point = random_point(m_map, m_seed);
        const auto nn = m_kd_tree.nearest_neighbor(new_point);
        Coordinate point;
        void *root_ptr;
        std::tie(point, root_ptr) = nn;
        Node *root = reinterpret_cast<Node *>(root_ptr);

        Node *new_node = can_reach(root->point, new_point);
        if (new_node == nullptr) continue;
        new_node->parent = root;
        m_nodes.push_back(new_node);
        m_kd_tree.add(new_node->point, reinterpret_cast<void *>(new_node));
        m_quad_tree.add(new_node->point, reinterpret_cast<void *>(new_node));

        const Coordinate neighborhood_start = {new_node->point.i - m_radius / 2,
                                               new_node->point.j - m_radius / 2};
        const Coordinate neighborhood_stop = {new_node->point.i + m_radius / 2,
                                              new_node->point.j + m_radius / 2};
        auto neighborhood = m_quad_tree.range_query(neighborhood_start, neighborhood_stop);
        assert(!neighborhood.empty());

        for (const auto &neighbor : neighborhood)
        {
            std::tie(point, root_ptr) = neighbor;
            root = reinterpret_cast<Node *>(root_ptr);

            const double dist = euclidean_distance(root->point, new_node->point);
            if (root->cost + dist >= new_node->cost) continue;

            Node *dummy = can_reach(root->point, new_node->point);
            if (dummy == nullptr) continue;
            delete dummy;

            new_node->parent = root;
            new_node->cost = root->cost + dist;
        }

        if (canvas != nullptr)
        {
            cv::line(*canvas,
                     {new_node->point.j, new_node->point.i},
                     {new_node->parent->point.j, new_node->parent->point.i},
                     BLUE,
                     1);
            cv::circle(*canvas, {new_node->point.j, new_node->point.i}, 2, RED, cv::FILLED);
        }

        for (const auto &neighbor : neighborhood)
        {
            std::tie(point, root_ptr) = neighbor;
            root = reinterpret_cast<Node *>(root_ptr);

            Node *dummy = can_reach(root->point, new_node->point);
            if (dummy == nullptr) continue;
            delete dummy;

            const double dist = euclidean_distance(root->point, new_node->point);
            if (root->cost <= new_node->cost + dist) continue;

            root->cost = new_node->cost + dist;
            if (canvas != nullptr)
            {
                cv::line(*canvas,
                         {root->parent->point.j, root->parent->point.i},
                         {root->point.j, root->point.i},
                         WHITE,
                         1);
                cv::line(*canvas,
                         {new_node->point.j, new_node->point.i},
                         {root->point.j, root->point.i},
                         BLUE,
                         1);
            }
            root->parent = new_node;
        }

        const double dist = euclidean_distance(new_node->point, m_B);
        if (dist <= m_reach && can_reach(new_node->point, m_B) != nullptr)
        {
            if (m_last_node == nullptr || m_last_node->cost > new_node->cost)
                m_last_node = new_node;

            m_success = true;

            return true;
        }

        return false;
    }

    log_error("Reached the maximum number of allowed states");
    m_used_up = true;

    return true;
}

std::vector<Coordinate> RRTStar::recover_path()
{
    if (!m_success) return {};

    std::vector<Coordinate> path;
    Node *curr = m_last_node;
    while (curr != nullptr)
    {
        path.push_back(curr->point);
        curr = curr->parent;
    }

    return path;
}

}  // namespace slam
