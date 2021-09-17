#include "mcl.h"
#include "motion.h"
#include "raycast.h"
#include "util.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>

constexpr int CANVAS_SIZE = 1000;
constexpr double OCCUPIED_THRESH = 0.5;

namespace slam
{
MCL::MCL(int n_particles, const std::array<double, 4> alphas) : m_alphas(alphas)
{
    this->particles.resize(n_particles);
    reset_particles();
}

void MCL::reset_particles()
{
    const auto blank_canvas =
        cv::Mat(cv::Size(CANVAS_SIZE, CANVAS_SIZE), CV_64F, cv::Scalar(0.5));
    const double uniform_weight = 1.0 / this->particles.size();
    const Pose center{0.5 * CANVAS_SIZE, 0.5 * CANVAS_SIZE, M_PI / 2};
    for (Particle& particle : this->particles)
    {
        particle.weight = uniform_weight;
        particle.pose = center;
        particle.map = blank_canvas.clone();
    }
}

// TODO : Could be moved to utils
Pose MCL::average_pose() const
{
    double avg_x = 0;
    double avg_y = 0;
    double theta_x = 0;
    double theta_y = 0;
    for (const Particle& particle : this->particles)
    {
        avg_x += particle.pose.x;
        avg_y += particle.pose.y;
        theta_x += std::cos(particle.pose.theta);
        theta_y += std::sin(particle.pose.theta);
    }

    avg_x /= this->particles.size();
    avg_y /= this->particles.size();
    const double avg_theta = std::atan2(theta_y, theta_x);

    return {avg_x, avg_y, avg_theta};
}

void MCL::predict(const Odometry& odom)
{
    for (Particle& particle : this->particles)
    {
        particle.pose =
            sample_motion_model_odometry(odom, particle.pose, this->m_alphas);
    }
}

void MCL::update(const Lidar& lidar, const std::vector<double>& scans)
{
    const double range = lidar.stop - lidar.start;
    const double step = range / lidar.n_rays;
    for (Particle& particle : this->particles)
    {
        double weight = 0;
        const Pose pose = particle.pose;
        particle.pose.theta -= range / 2;
        for (int i = 0; i < lidar.n_rays; ++i)
        {
            double w = measurement_model_beam(scans[i], lidar.stddev, particle,
                                              lidar.max_dist);
            weight += std::log(w);
            particle.pose.theta += step;
        }
        particle.weight = std::exp(weight);
        particle.pose = pose;
    }

    resample_particles();
}

std::vector<Particle> MCL::probabilistic_fitness_selection(int n)
{
    // O(nlog m) where m = particles.size()
    double sum = 0;
    for (Particle& particle : this->particles)
    {
        sum += particle.weight;
    }

    static thread_local std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> distribution(0, sum);

    std::vector<double> cumulative_sums;
    cumulative_sums.reserve(this->particles.size());
    cumulative_sums.push_back(this->particles.front().weight);
    for (unsigned i = 1; i < this->particles.size(); ++i)
    {
        cumulative_sums.push_back(cumulative_sums.back() +
                                  this->particles[i].weight);
    }

    std::vector<Particle> new_particles;
    new_particles.reserve(n);
    for (int k = 0; k < n; ++k)
    {
        const double cutoff = distribution(generator);
        int i = 0;
        int j = cumulative_sums.size() - 1;
        while (i < j)
        {
            const int mid = i + (j - i) / 2;
            if (cumulative_sums[mid] == cutoff)
            {
                i = mid;
                j = mid;
            }
            else if (cumulative_sums[mid] < cutoff)
            {
                i = mid + 1;
            }
            else
            {
                j = mid;
            }
        }

        Particle copy = this->particles[i];
        copy.map = this->particles[i].map.clone();
        new_particles.push_back(copy);
    }

    return new_particles;
}

void MCL::resample_particles()
{
    std::vector<Particle> new_particles =
        probabilistic_fitness_selection(this->particles.size());
    std::swap(new_particles, this->particles);
    std::sort(this->particles.begin(), this->particles.end(),
              [](const Particle& lhs, const Particle& rhs) {
                  return lhs.weight > rhs.weight;
              });
}

}  // namespace slam
