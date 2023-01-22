#include "hastar.h"

#include <cmath>

#include "colors.h"
#include "raycast.h"

namespace slam
{
HybridAStar::HybridAStar(const cv::Mat& map,
                         const Pose& A,
                         const Pose& B,
                         double v,
                         double theta,
                         double length,
                         unsigned theta_res,
                         int branching_factor,
                         double tol,
                         bool diff_drive)
{
    reset(map, A, B, v, theta, length, theta_res, branching_factor, tol, diff_drive);
    m_comp = [this](const Node& X, const Node& Y)
    { return X.cost + X.dist_to_target > Y.cost + Y.dist_to_target; };
}

// Optimization: Accounts for 6% of the runtime. Worse if the path is easy to find. This is because
// we reset all values in the cuboid. We can use a sequence number instead to avoid having to reset
// all of them.

void HybridAStar::reset(const cv::Mat& map,
                        const Pose& A,
                        const Pose& B,
                        double v,
                        double theta,
                        double length,
                        unsigned theta_res,
                        int branching_factor,
                        double tol,
                        bool diff_drive)
{
    m_success = false;
    m_used_up = false;
    m_A = A;
    m_B = B;
    m_map = map;
    m_size = 0;
    m_v = v;
    m_theta = theta;
    m_length = length;
    m_theta_res = theta_res;
    m_tol = tol * tol;
    m_diff_drive = diff_drive;

    m_cuboid.clear();
    m_cuboid.resize(map.rows * map.cols * theta_res, CuboidEntry());

    m_target = pose_to_cuboid_index(B);

    const CuboidIndex start = pose_to_cuboid_index(A);
    m_q.clear();
    m_q.reserve(MAX_QUEUE_SIZE);
    m_q.push_back({A, 0.0, euclidean_distance(A, B), start});
    std::push_heap(m_q.begin(), m_q.end(), m_comp);

    assert(branching_factor > 2);
    assert(branching_factor % 2 == 1);

    const double COST_SLOPE = v / (branching_factor - 1);
    const double dtheta = theta * 2 / (branching_factor - 1);
    const int mid = branching_factor / 2;
    m_thetas.clear();
    m_steering_costs.clear();
    for (int i = 0; i < branching_factor; ++i)
    {
        m_thetas.push_back(-theta + i * dtheta);
        m_steering_costs.push_back(std::abs(i - mid) * COST_SLOPE);
    }

    m_velocities[0] = v;
    m_velocities[1] = -v;
}

// Optimization [1.54]: vel, m_length and angle are all known values. They should be precomputed.

// Optimization [~6]: All angle offsets are known ahead of time. Also, given that there is an angle
// resolution for memoization, sine and cosine functions can be precomputed.

std::vector<std::pair<Pose, double>> HybridAStar::steering_adjacency(const Pose& pose) const
{
    std::vector<std::pair<Pose, double>> neighborhood;
    neighborhood.reserve(m_velocities.size() * m_thetas.size());

    constexpr double REVERSE_FACTOR = 10;
    for (double vel : m_velocities)
    {
        const double cost_factor = vel < 0 ? REVERSE_FACTOR : 1;  // more costly to go backwards
        for (unsigned i = 0; i < m_thetas.size(); ++i)
        {
            const double steer = m_thetas[i];
            const double cost = m_steering_costs[i];

            Pose new_pose = pose;
            new_pose.theta += vel / m_length * std::tan(steer);
            new_pose.x += vel * std::cos(new_pose.theta);
            new_pose.y += vel * std::sin(new_pose.theta);

            neighborhood.emplace_back(new_pose, m_v + cost * cost_factor);
        }
    }

    return neighborhood;
}

// Optimization [3.33 + 2.37 = 5.7]: We are computing the x and y displacements to do an `atan2`
// operation and retrieve the angle. This is pointless given that we are calling sin and cos to
// retrieve dx and dy again.

// Optimization: The main loop of the raycast algorithm can be speeded up by a factor of the length
// of SIMD instructions. We can look at many cells at a time instead of doing it in a scalar way.

// Optimization: The raycast checks if the indices are in boundaries. We can filter that check by
// giving a maximum distance that does not make it go out of boundaries.
bool HybridAStar::can_reach(const Pose& src, const Pose& dst) const
{
    // Assuming src is a valid starting point
    const Coordinate dst_coord = pose_to_image_coordinates_(m_map, dst);
    if (!within_boundaries(m_map, dst_coord)) return false;
    if (m_map.at<unsigned char>(dst_coord.i, dst_coord.j) < 0.5) return false;

    const double dist_squared = euclidean_distance_squared(src, dst);
    const double dx = dst.x - src.x;
    const double dy = dst.y - src.y;
    const double angle = std::atan2(dy, dx);

    Pose raycast_pose = src;
    raycast_pose.theta = angle;

    // FIXME : Use circular raycast
    return raycast<unsigned char>(m_map, raycast_pose, dist_squared).x == -1;
}

// Optimization: Use https://github.com/skarupke/heap/blob/master/minmax_and_dary_heap.hpp to
// speedup heap operations. This will likely have a 2x speedup in heap pop operations.

// Optimization: Before adding an entry in the heap, check if it will not be immediately invalidated
// when taken from the heap.

// Optimization: Add a straight line checker (single raycast).

// Optimization: Use a different heuristic.

bool HybridAStar::pathfind(cv::Mat* canvas)
{
    if (m_success || m_used_up)
    {
        log_error("HA* object must now be reset or deallocated");
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
        CuboidEntry& entry = cuboid_at(X_index);
        if (X.cost >= entry.cost) continue;
        entry.cost = X.cost;
        entry.parent = X.parent;

        const bool within_tol = euclidean_distance_squared(X.pose, m_B) <= m_tol;
        if (within_tol)
        {
            m_last_node = X_index;
            m_success = m_diff_drive || X_index.k == m_target.k;
            if (m_success) return true;
        }

        for (const auto& neighbor : steering_adjacency(X.pose))
        {
            Pose pose;
            double cost;
            std::tie(pose, cost) = neighbor;

            if (!can_reach(X.pose, pose)) continue;
            const CuboidIndex Y_index = pose_to_cuboid_index(pose);
            if (X.cost + cost >= cuboid_at(Y_index).cost) continue;

            m_q.emplace_back(pose, X.cost + cost, euclidean_distance(pose, m_B), X_index);
            std::push_heap(m_q.begin(), m_q.end(), m_comp);

            if (canvas)
            {
                const CuboidIndex Y_index = pose_to_cuboid_index(pose);
                cv::line(*canvas, {X_index.j, X_index.i}, {Y_index.j, Y_index.i}, BLUE, 1);
            }
        }

        ++m_size;

        return false;
    }

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
        next = cuboid_at(next).parent;
    }

    std::reverse(path.begin(), path.end());

    return path;
}

HybridAStar::CuboidIndex HybridAStar::pose_to_cuboid_index(const Pose& pose) const
{
    const Coordinate coord = pose_to_image_coordinates_(m_map, pose);
    const double angle = std::fmod(pose.theta + 2 * M_PI, 2 * M_PI);
    const int angle_index = angle / (2 * M_PI / m_theta_res);

    return {coord.i, coord.j, angle_index};
}

}  // namespace slam
