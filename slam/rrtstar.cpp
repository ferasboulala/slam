#include "rrtstar.h"
#include "colors.h"

namespace slam
{
RRTStar::RRTStar(const cv::Mat &map, const Coordinate &A, const Coordinate &B,
                 double radius)
    : m_A(A),
      m_B(B),
      m_map(map),
      m_root(new Node{A, nullptr, {}}),
      m_last_node(nullptr),
      m_size(0),
      m_radius(radius)
{
    m_tree.add(A, m_root);
}

void free(RRTStar::Node *root)
{
    if (root == nullptr) return;

    for (RRTStar::Node *child : root->children)
    {
        free(child);
    }

    delete root;
}

RRTStar::~RRTStar() { free(m_root); }
bool RRTStar::pathfind(cv::Mat *canvas)
{
    while (true)
    {
        const Coordinate new_point = random_point(m_map);
        const auto nn_obj = m_tree.nearest_neighbor(new_point);
        Coordinate nn;
        void *root_ptr;
        std::tie(nn, root_ptr) = nn_obj;
        Node *root = reinterpret_cast<Node *>(root_ptr);

        double dist = euclidean_distance(root->point, new_point);
        Coordinate new_point_to_add = new_point;
        if (dist > m_radius)
        {
            const double di = (new_point.i - root->point.i) / dist;
            const double dj = (new_point.j - root->point.j) / dist;
            new_point_to_add =
                Coordinate{static_cast<int>(root->point.i + di * m_radius),
                           static_cast<int>(root->point.j + dj * m_radius)};
        }

        Node *new_node = new Node{new_point_to_add, root, {}};
        root->children.push_back(new_node);
        m_tree.add(new_point_to_add, reinterpret_cast<void *>(new_node));
        ++m_size;

        if (canvas != nullptr)
        {
            cv::line(*canvas, {new_point_to_add.j, new_point_to_add.i},
                     {root->point.j, root->point.i}, BLUE, 2);
            cv::circle(*canvas, {new_point_to_add.j, new_point_to_add.i}, 3,
                       RED, cv::FILLED);
        }

        dist = euclidean_distance(new_point_to_add, m_B);
        if (dist <= m_radius)
        {
            m_last_node = new_node;
            return true;
        }

        return false;
    }
}

std::vector<Coordinate> RRTStar::recover_path()
{
    if (!m_success) return {};

    std::vector<Coordinate> path;
    while (m_last_node->parent != m_root)
    {
        path.push_back(m_last_node->point);
        m_last_node = m_last_node->parent;
    }

    return path;
}

}  // namespace slam