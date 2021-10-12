#pragma once

#include "pose.h"
#include "thirdparty/log.h"
#include "util.h"

#include <array>
#include <functional>
#include <limits>
#include <vector>

namespace slam
{
class HybridAStar
{
public:
    HybridAStar(cv::Mat& map, const Pose& A, const Pose& B, double v, double theta, double length = 1,
                int branching_factor = 3, bool m_diff_drive = true);

    ~HybridAStar() = default;

    bool pathfind(cv::Mat* canvas);
    std::vector<Coordinate> recover_path();
    unsigned size() const { return m_size; }
private:
    struct CuboidIndex
    {
        int i;
        int j;
        int k;
        inline bool operator==(const CuboidIndex& other) const { return i == other.i && j == other.j && k == other.k; }
        inline bool operator!=(const CuboidIndex& other) const { return !(*this == other); }
    };

    CuboidIndex pose_to_cuboid_index(const Pose& pose) const;
    bool can_reach(const Pose& src, const Pose& dst) const;
    std::vector<std::pair<Pose, double>> steering_adjacency(const Pose& pose) const;

    struct Node
    {
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

    const cv::Mat& m_map;

    unsigned m_size;
    double m_v;
    double m_theta;
    double m_length;
    bool m_diff_drive;

    // Performance reasons
    std::array<double, 2> m_velocities;
    std::vector<double> m_thetas;
    std::vector<double> m_steering_costs;

    using Grid = std::vector<std::vector<std::pair<double, CuboidIndex>>>;
    using Cuboid = std::vector<Grid>;
    Cuboid m_costs;
    std::vector<Node> m_q;  // point, orientation, cost
    std::function<bool(const Node& X, const Node& Y)> m_comp;
};

}  // namespace slam