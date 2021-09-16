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
    MCL(int n_particles, const std::array<double, 4> alphas, double alpha_fast = 0.9,
        double alpha_slow = 0.1);
    ~MCL() = default;

    void predict(const Odometry& odom);
    void update(const Lidar& lidar, const std::vector<double>& scans);

    Pose average_pose() const;

private:
    // Creates random particles.
    void reset_particles();
    // Filters out particles on obstacles and outside the map.
    void filter_particles();
    // Selects the best fit particles.
    void resample_particles();
    // Updates the global map.
    void map();

    std::vector<Particle> fitness_selection(int n);
    std::vector<Particle> probabilistic_fitness_selection(int n);
    int compute_number_new_particles();

public:
    std::vector<Particle> particles;
    cv::Mat occupancy_grid;

private:
    cv::Mat m_occupied;
    cv::Mat m_free;

    Box m_bounding_box;

    std::array<double, 4> m_alphas;
    double m_omega_fast;
    double m_omega_slow;
    double m_alpha_fast;
    double m_alpha_slow;
};

}  // namespace slam
