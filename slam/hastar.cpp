#include "hastar.h"
#include "raycast.h"

namespace slam
{
HybridAStar::HybridAStar(cv::Mat& map, const Pose& A, const Pose& B, double v, double theta, double length, unsigned theta_res, unsigned branching_factor)
    : m_success(false), m_used_up(false), m_A(A), m_B(B), m_map(map), m_size(0), m_v(v), m_theta(theta), m_length(length), m_branching_factor(branching_factor)
{
    m_target = pose_to_cuboid_index(B);
    m_costs = Cuboid(map.rows, Grid(map.cols, std::vector<double>(theta_res, std::numeric_limits<double>::max())));

    m_heuristic = [this](const Pose& X) { return euclidean_distance(X, m_B); };

    m_comp = [this](const Node& X, const Node& Y) {
        return X.cost + m_heuristic(X.pose) > Y.cost + m_heuristic(Y.pose);
    };

    m_q.push_back({A, 0.0});
    std::push_heap(m_q.begin(), m_q.end(), m_comp);
}

static std::vector<std::pair<Pose, double>> steering_adjacency(const Pose &pose, double v, double theta, double L, unsigned branching_factor)
{
    constexpr double EPSILON = 1e-6;

    assert(branching_factor > 2);
    assert(branching_factor % 2 == 1 && "Branching factor must be odd");

    std::array<double, 2> velocities = {v, -v};
    std::vector<double> thetas;
    std::vector<double> costs;

    const double COST_SLOPE = 1 / (branching_factor - 1);
    const double dtheta = theta * 2 / (branching_factor - 1);
    const unsigned mid = branching_factor / 2 + 1;
    for (unsigned i = 0; i < branching_factor; ++i)
    {
        thetas.push_back(-theta + i * dtheta);
        costs.push_back(std::abs(i - mid) * COST_SLOPE);
    }

    std::vector<std::pair<Pose, double>> neighborhood;
    for (double vel : velocities)
    {
        const double cost_factor = vel < 0 ? 2 : 1; // more costly to go backwards
        for (unsigned i = 0; i < thetas.size(); ++i)
        {
            const double angle = thetas[i];
            const double cost = costs[i];

            Pose new_pose = pose;
            new_pose.theta += vel / L * std::tan(angle + EPSILON);
            new_pose.x += vel * std::cos(new_pose.theta);
            new_pose.y += vel * std::sin(new_pose.theta);

            neighborhood.push_back({new_pose, cost * cost_factor});
        }
    }

    return neighborhood;
}

bool HybridAStar::can_reach(const Pose &src, const Pose &dst) const
{
    const Coordinate dst_coord = pose_to_image_coordinates_(m_map, dst);
    if (!within_boundaries(m_map, dst_coord)) return false;
    if (m_map.at<double>(dst_coord.i, dst_coord.j)) return false;

    const double dist = euclidean_distance(src, dst);
    const double dx = dst.x - src.x;
    const double dy = dst.y - src.y;
    const double angle = std::atan2(dy, dx);

    Pose raycast_pose = src;
    raycast_pose.theta = angle;

    return raycast<double>(m_map, raycast_pose, dist).x == -1;
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

        const Coordinate X_coord = pose_to_image_coordinates_(m_map, X.pose);
        if (!within_boundaries(m_map, X_coord)) continue;

        if (m_map.at<double>(X_coord.i, X_coord.j) < 0.5) continue;

        const CuboidIndex X_index = pose_to_cuboid_index(X.pose);
        if (X_index == m_target)
        {
            m_success = true;
            return true;
        }

        if (X.cost >= m_costs[X_index.i][X_index.j][X_index.k]) continue;

        m_costs[X_index.i][X_index.j][X_index.k] = X.cost;

         for (const auto &neighbor : steering_adjacency(X.pose, m_v, m_theta, m_length, m_branching_factor))
         {
             Pose pose;
             double cost;
             std::tie(pose, cost) = neighbor;

             if (!can_reach(X.pose, pose)) continue;
             m_q.push_back({pose, X.cost + cost});
             std::push_heap(m_q.begin(), m_q.end(), m_comp);
         }

        ++m_size;

        return false;
    }

    m_success = false;
    m_used_up = true;

    return true;
}

std::vector<Coordinate> HybridAStar::recover_path() { return {}; }


HybridAStar::CuboidIndex HybridAStar::pose_to_cuboid_index(const Pose &pose) const
{
    const Coordinate coord = pose_to_image_coordinates_(m_map, pose);
    const double angle = normalize_angle(pose.theta + 2 * M_PI);
    const int angle_index = angle / (2 * M_PI / m_costs.front().front().size());

    return {coord.i, coord.j, angle_index};
}

};  // namespace slam