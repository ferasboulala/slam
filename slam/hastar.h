#pragma once

#include <array>
#include <functional>
#include <limits>
#include <vector>

#include "pose.h"
#include "thirdparty/log.h"
#include "util.h"

namespace slam
{
class HybridAStar
{
public:
    HybridAStar(const cv::Mat& map,
                const Pose& A,
                const Pose& B,
                double v,
                double theta,
                double length,
                unsigned theta_res,
                int branching_factor,
                double tol,
                bool m_diff_drive = true);
    HybridAStar(const HybridAStar& other) = delete;

    ~HybridAStar() = default;

    void reset(const cv::Mat& map,
               const Pose& A,
               const Pose& B,
               double v,
               double theta,
               double length,
               unsigned theta_res,
               int branching_factor,
               double tol,
               bool m_diff_drive = true);
    bool pathfind(cv::Mat* canvas);
    std::vector<Coordinate> recover_path();
    inline unsigned size() const { return m_size; }

private:
    struct CuboidIndex
    {
        int i;
        int j;
        int k;
        inline bool operator==(const CuboidIndex& other) const
        {
            return i == other.i && j == other.j && k == other.k;
        }
        inline bool operator!=(const CuboidIndex& other) const { return !(*this == other); }
    };

    struct CuboidEntry
    {
        CuboidEntry() { cost = std::numeric_limits<double>::max(); }
        double cost;
        CuboidIndex parent;
    };

    inline CuboidEntry& cuboid_at(const CuboidIndex& index)
    {
        const unsigned unraveled_index =
            index.i * m_map.cols * m_theta_res + index.j * m_theta_res + index.k;

        return m_cuboid[unraveled_index];
    }

    CuboidIndex pose_to_cuboid_index(const Pose& pose) const;
    bool can_reach(const Pose& src, const Pose& dst) const;
    std::vector<std::pair<Pose, double>> steering_adjacency(const Pose& pose) const;

    struct Node
    {
        Node(const Pose& p, double c, double d, const CuboidIndex& i)
        {
            pose = p;
            cost = c;
            dist_to_target = d;
            parent = i;
        }
        Pose pose;
        double cost;
        double dist_to_target;
        CuboidIndex parent;
    };
    static const unsigned MAX_QUEUE_SIZE = 1e9 / sizeof(Node);  // 1 gigabyte

    bool m_success;
    bool m_used_up;

    Pose m_A;
    Pose m_B;
    CuboidIndex m_target;
    CuboidIndex m_last_node;

    cv::Mat m_map;

    unsigned m_size;
    double m_v;
    double m_theta;
    double m_length;
    unsigned m_theta_res;
    double m_tol;
    bool m_diff_drive;

    // Performance reasons
    std::array<double, 2> m_velocities;
    std::vector<double> m_thetas;
    std::vector<double> m_steering_costs;

    std::vector<CuboidEntry> m_cuboid;
    std::vector<Node> m_q;  // point, orientation, cost
    std::function<bool(const Node& X, const Node& Y)> m_comp;
};

}  // namespace slam
