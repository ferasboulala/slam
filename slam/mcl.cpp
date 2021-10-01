#include "mcl.h"
#include "motion.h"
#include "raycast.h"
#include "util.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <random>
#include <thread>

constexpr int CANVAS_SIZE = 1000;

namespace slam
{
MCL::MCL(const Lidar &lidar, int n_particles,
         const std::array<double, 4> alphas)
    : lidar(lidar), m_alphas(alphas)
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
    for (Particle &particle : this->particles)
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
    for (const Particle &particle : this->particles)
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

void MCL::predict(const Odometry &odom)
{
    for (Particle &particle : this->particles)
    {
        particle.pose =
            sample_motion_model_odometry(odom, particle.pose, this->m_alphas);
    }
}

void MCL::update_inner(const std::vector<double> &scans, int start, int n)
{
    const double range = this->lidar.stop - this->lidar.start;
    const double step = range / lidar.n_rays;
    for (int i = 0; i < n; ++i)
    {
        Particle &particle = this->particles[i + start];
        double weight = 0;
        const Pose pose = particle.pose;
        particle.pose.theta -= range / 2;
        for (int j = 0; j < this->lidar.n_rays; ++j)
        {
            double w = measurement_model_beam(scans[j], this->lidar.stddev,
                                              particle, this->lidar.max_dist);
            weight += std::log(w);
            particle.pose.theta += step;
        }
        particle.weight = std::exp(weight);
        particle.pose = pose;
    }
}

void MCL::update(const std::vector<double> &scans)
{
    assert(scans.size() == this->lidar.n_rays &&
           "Number of reported scans do not match lidar number of rays");

    std::vector<std::thread> threads(std::thread::hardware_concurrency());
    const int number_particles_per_thread =
        this->particles.size() / threads.size();
    for (unsigned i = 0; i < threads.size(); ++i)
    {
        const int start = i * number_particles_per_thread;
        int n = number_particles_per_thread;
        if (i == threads.size() - 1)
            n += this->particles.size() % threads.size();

        threads[i] = std::thread(&MCL::update_inner, this, scans, start, n);
    }

    for (std::thread &thread : threads)
    {
        thread.join();
    }

    resample_particles();
}

std::vector<Particle> MCL::probabilistic_fitness_selection(int n)
{
    // O(nlog m) where m = particles.size()
    double sum = 0;
    for (Particle &particle : this->particles)
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
              [](const Particle &lhs, const Particle &rhs) {
                  return lhs.weight > rhs.weight;
              });
}

}  // namespace slam
