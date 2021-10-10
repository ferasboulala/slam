#include "hastar.h"

namespace slam
{
HybridAStar::HybridAStar(cv::Mat& map, const Pose& A, const Pose& B, double v, double theta, unsigned theta_res)
    : m_success(false), m_used_up(false), m_A(A), m_B(B), m_map(map), m_size(0), m_v(v), m_theta(theta)
{
    m_distances = Cuboid(map.rows, Grid(map.cols, std::vector<double>(theta_res, std::numeric_limits<double>::max())));

    m_heuristic = [this](const Pose& X) { return euclidean_distance(X, m_B); };

    m_comp = [this](const Node& X, const Node& Y) {
        return X.cost + m_heuristic(X.pose) > Y.cost + m_heuristic(Y.pose);
    };

    m_q.push_back({A, 0.0});
    std::push_heap(m_q.begin(), m_q.end(), m_comp);
}

bool HybridAStar::pathfind(cv::Mat*)
{
    if (m_used_up)
    {
        log_error("HA* object must now be deallocated");
        return true;
    }

    Node X;
    while (!m_q.empty())
    {
        if (m_q.size() > MAX_QUEUE_SIZE)
        {
            log_warn("A* pathfinder exceeded queue size");
            return true;
        }

        X = m_q.front();
        std::pop_heap(m_q.begin(), m_q.end(), m_comp);
        m_q.pop_back();

        // if (!within_boundaries(m_map, X)) continue;

        // if (m_map.at<double>(X.i, X.j) < 0.5) continue;

        // if (X == m_B)
        // {
        //     m_success = true;
        //     return true;
        // }

        // if (canvas)
        // {
        //     cv::Vec3f& color = canvas->at<cv::Vec3f>(X.i, X.j);
        //     color[0] = BLUE[0];
        //     color[1] = BLUE[1];
        //     color[2] = BLUE[2];
        // }

        // if (dist >= m_distances.at<double>(X.i, X.j)) continue;

        // m_distances.at<double>(X.i, X.j) = dist;

        // for (const Coordinate& coord : adjacency_8(X))
        // {
        //     double new_dist = dist;
        //     if (coord.i != X.i && coord.j != X.j)
        //         new_dist += std::sqrt(2.0);
        //     else
        //         new_dist += 1;

        //     m_q.push_back(std::tuple<Coordinate, double>(coord, new_dist));
        //     std::push_heap(m_q.begin(), m_q.end(), m_comp);
        // }

        // ++m_size;

        return false;
    }

    m_success = false;
    m_used_up = true;

    return true;
}

std::vector<Coordinate> HybridAStar::recover_path() { return {}; }
};  // namespace slam