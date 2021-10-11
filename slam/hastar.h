#pragma once

#include "pose.h"
#include "thirdparty/log.h"
#include "util.h"

#include <functional>
#include <limits>
#include <vector>

namespace slam
{
class HybridAStar
{
public:
    HybridAStar(cv::Mat& map, const Pose& A, const Pose& B, double v, double theta, double length = 1, unsigned theta_res = 10, unsigned branching_factor = 3);

    ~HybridAStar() = default;

    bool pathfind(cv::Mat* canvas);
    std::vector<Coordinate> recover_path();

private:
    struct CuboidIndex
    {
        int i;
        int j;
        int k;
        inline bool operator==(const CuboidIndex& other) const { return i == other.i && j == other.j && k == other.k; }
    };

    CuboidIndex pose_to_cuboid_index(const Pose &pose) const;
    bool can_reach(const Pose &src, const Pose &dst) const;

    struct Node
    {
        Pose pose;
        double cost;
    };
    static const unsigned MAX_QUEUE_SIZE = 1e9 / sizeof(Node);  // 1 gigabyte

    bool m_success;
    bool m_used_up;

    Pose m_A;
    Pose m_B;
    CuboidIndex m_target;

    const cv::Mat& m_map;

    unsigned m_size;
    double m_v;
    double m_theta;
    double m_length;
    unsigned m_branching_factor;

    using Grid = std::vector<std::vector<double>>;
    using Cuboid = std::vector<Grid>;
    Cuboid m_costs;
    std::vector<Node> m_q;  // point, orientation, cost
    std::function<double(const Pose& X)> m_heuristic;
    std::function<bool(const Node& X, const Node& Y)> m_comp;
};

}  // namespace slam