#include "hastar.h"
#include "colors.h"
#include "raycast.h"

#include <cmath>

namespace slam
{
HybridAStar::HybridAStar(cv::Mat& map, const Pose& A, const Pose& B, double v, double theta, double length,
                         int branching_factor, bool diff_drive)
    : m_success(false),
      m_used_up(false),
      m_A(A),
      m_B(B),
      m_map(map),
      m_size(0),
      m_v(v),
      m_theta(theta),
      m_length(length),
      m_diff_drive(diff_drive)
{
    assert(theta < M_PI / 2 && "Theta must be small enough for a sufficient resolution");
    const unsigned theta_res = 2 * M_PI / theta;
    m_costs = Cuboid(map.rows, Grid(map.cols, std::vector<std::pair<double, CuboidIndex>>(
                                                  theta_res, {std::numeric_limits<double>::max(), {-1, -1, -1}})));

    m_target = pose_to_cuboid_index(B);

    m_comp = [this](const Node& X, const Node& Y) { return X.cost + X.dist_to_target > Y.cost + Y.dist_to_target; };

    const CuboidIndex start = pose_to_cuboid_index(A);
    m_q.push_back({A, 0.0, euclidean_distance(A, B), start});
    std::push_heap(m_q.begin(), m_q.end(), m_comp);

    assert(branching_factor > 2);
    assert(branching_factor % 2 == 1);

    const double COST_SLOPE = v / (branching_factor - 1);
    const double dtheta = theta * 2 / (branching_factor - 1);
    const int mid = branching_factor / 2;
    for (int i = 0; i < branching_factor; ++i)
    {
        m_thetas.push_back(-theta + i * dtheta);
        m_steering_costs.push_back(std::abs(i - mid) * COST_SLOPE);
    }

    m_velocities[0] = v;
    m_velocities[1] = -v;
}

std::vector<std::pair<Pose, double>> HybridAStar::steering_adjacency(const Pose& pose) const
{
    std::vector<std::pair<Pose, double>> neighborhood;
    constexpr double REVERSE_FACTOR = 2;
    for (double vel : m_velocities)
    {
        const double cost_factor = vel < 0 ? REVERSE_FACTOR : 1;  // more costly to go backwards
        for (unsigned i = 0; i < m_thetas.size(); ++i)
        {
            const double angle = m_thetas[i];
            const double cost = m_steering_costs[i];

            Pose new_pose = pose;
            new_pose.theta += vel / m_length * std::tan(angle);
            new_pose.x += vel * std::cos(new_pose.theta);
            new_pose.y += vel * std::sin(new_pose.theta);

            neighborhood.push_back({new_pose, m_v + cost * cost_factor});
        }
    }

    return neighborhood;
}

bool HybridAStar::can_reach(const Pose& src, const Pose& dst) const
{
    // Assuming src is a valid starting point
    const Coordinate dst_coord = pose_to_image_coordinates_(m_map, dst);
    if (!within_boundaries(m_map, dst_coord)) return false;
    if (m_map.at<double>(dst_coord.i, dst_coord.j) < 0.5) return false;

    const double dist = euclidean_distance(src, dst);
    const double dx = dst.x - src.x;
    const double dy = dst.y - src.y;
    const double angle = std::atan2(dy, dx);

    Pose raycast_pose = src;
    raycast_pose.theta = angle;

    // FIXME : Use circular raycast using this formula until we reach the point
    return raycast<double>(m_map, raycast_pose, dist).x == -1;
}

bool HybridAStar::pathfind(cv::Mat*)
{
    if (m_success || m_used_up)
    {
        log_error("HA* object must now be deallocated");
        return true;
    }

    while (!m_q.empty())
    {
        if (m_q.size() > MAX_QUEUE_SIZE)
        {
            log_warn("A* pathfinder exceeded queue size");
            return true;
        }

        Node X = m_q.front();
        std::pop_heap(m_q.begin(), m_q.end(), m_comp);
        m_q.pop_back();

        const CuboidIndex X_index = pose_to_cuboid_index(X.pose);
        if (X.cost >= std::get<0>(m_costs[X_index.i][X_index.j][X_index.k])) continue;
        m_costs[X_index.i][X_index.j][X_index.k] = {X.cost, X.parent};

        if (m_diff_drive)
            m_success = pose_to_image_coordinates_(m_map, X.pose) == pose_to_image_coordinates_(m_map, m_B);
        else if (X_index == m_target)
            m_success = true;

        if (m_success)
        {
            m_last_node = X_index;
            return true;
        }

        for (const auto& neighbor : steering_adjacency(X.pose))
        {
            Pose pose;
            double cost;
            std::tie(pose, cost) = neighbor;

            if (!can_reach(X.pose, pose)) continue;
            m_q.push_back({pose, X.cost + cost, euclidean_distance(pose, m_B), X_index});
            std::push_heap(m_q.begin(), m_q.end(), m_comp);
        }

        ++m_size;

        return false;
    }

    m_success = false;
    m_used_up = true;

    return true;
}

std::vector<Coordinate> HybridAStar::recover_path()
{
    if (!m_success) return {};

    std::vector<Coordinate> path;
    CuboidIndex next = m_last_node;
    const CuboidIndex start = pose_to_cuboid_index(m_A);
    while (next != start)
    {
        path.push_back({next.i, next.j});
        next = std::get<1>(m_costs[next.i][next.j][next.k]);
    }

    std::reverse(path.begin(), path.end());

    return path;
}

HybridAStar::CuboidIndex HybridAStar::pose_to_cuboid_index(const Pose& pose) const
{
    const Coordinate coord = pose_to_image_coordinates_(m_map, pose);
    const double angle = std::fmod(std::fmod(pose.theta, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
    const int angle_index = angle / (2 * M_PI / m_costs.front().front().size());

    return {coord.i, coord.j, angle_index};
}

};  // namespace slam