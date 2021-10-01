#pragma once

#include "lidar.h"
#include "pose.h"

#include <opencv2/opencv.hpp>

#include <array>
#include <vector>

namespace slam
{
class MCL
{
public:
    MCL(const Lidar &lidar, int n_particles,
        const std::array<double, 4> alphas);
    ~MCL() = default;

    void predict(const Odometry &odom);
    void update(const std::vector<double> &scans);

    Pose average_pose() const;

private:
    // Creates random particles.
    void reset_particles();
    // Filters out particles on obstacles and outside the map.
    void filter_particles();
    // Selects the best fit particles.
    void resample_particles();

    std::vector<Particle> fitness_selection(int n);
    std::vector<Particle> probabilistic_fitness_selection(int n);

    void update_inner(const std::vector<double> &scans, int start, int n);

public:
    std::vector<Particle> particles;
    const Lidar &lidar;

private:
    std::array<double, 4> m_alphas;
};

}  // namespace slam
