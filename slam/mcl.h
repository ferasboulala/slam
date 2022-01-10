#pragma once

#include <array>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

#include "pose.h"

namespace slam
{
class MCL
{
public:
    MCL(int n_particles, const cv::Size& canvas_size = {1000, 1000});
    ~MCL() = default;

    void predict(const Odometry& odom, const std::array<double, 4>& alphas);
    void update(const std::vector<std::tuple<double, double>>& scan,
                double stddev,
                double max_dist,
                const Pose& scanner_offset = {0, 0, 0},
                int n_threads = -1);

    const std::vector<Particle>& get_particles() const { return m_particles; }

    static Pose sensor_position(const Pose& frame_position, const Pose& scanner_offset);

private:
    void reset_particles();
    void filter_particles();
    void resample_particles();

    std::vector<Particle> fitness_selection(int n);
    std::vector<Particle> probabilistic_fitness_selection(int n);

    void update_inner(const std::vector<std::tuple<double, double>>& scan,
                      double stddev,
                      double max_dist,
                      int start,
                      int n,
                      const std::tuple<double, double, double>& scanner_offset);

    std::vector<Particle> m_particles;
    cv::Size m_canvas_size;
};

}  // namespace slam
