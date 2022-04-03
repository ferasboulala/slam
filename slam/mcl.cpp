#include "mcl.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <random>
#include <thread>

#include "motion.h"
#include "raycast.h"
#include "util.h"

namespace slam
{
MCL::MCL(int n_particles, const cv::Size& canvas_size) : m_canvas_size(canvas_size)
{
    m_particles.resize(n_particles);
    reset_particles();
}

Pose MCL::starting_pose() const
{
    return {m_canvas_size.width / 2.0, m_canvas_size.height / 2.0, M_PI / 2};
}

void MCL::reset_particles()
{
    // Maps are quantized
    const auto blank_canvas = cv::Mat(m_canvas_size, CV_8UC1, cv::Scalar(128));
    const double uniform_weight = 1.0 / m_particles.size();
    const Pose center = starting_pose();
    for (Particle& particle : m_particles)
    {
        particle.weight = uniform_weight;
        particle.pose = center;
        particle.map = blank_canvas.clone();
    }
}

void MCL::predict(const Odometry& odom, const std::array<double, 4>& alphas)
{
    for (Particle& particle : m_particles)
    {
        particle.pose = sample_motion_model_odometry(odom, particle.pose, alphas);
    }
}

void MCL::update_inner(const std::vector<std::tuple<double, double>>& scan,
                       double stddev,
                       double max_dist,
                       int start,
                       int n,
                       const std::tuple<double, double, double>& scanner_offset)
{
    const auto& [scanner_d, scanner_theta, scanner_rot] = scanner_offset;
    for (int i = 0; i < n; ++i)
    {
        Particle& particle = m_particles[i + start];
        const double dx = std::cos(particle.pose.theta + scanner_theta) * scanner_d;
        const double dy = std::sin(particle.pose.theta + scanner_theta) * scanner_d;
        const Pose pose = particle.pose;

        // w.r.t the sensor frame
        particle.pose.x += dx;
        particle.pose.y += dy;
        double weight = 0;
        const double max_dist_squared = max_dist * max_dist;
        for (const auto& [angle, dist] : scan)
        {
            particle.pose.theta = pose.theta + angle + scanner_rot;
            double w = measurement_model_beam(dist * dist, stddev, particle, max_dist_squared);
            weight += std::log(w);
        }
        particle.weight = std::exp(weight);
        particle.pose = pose;
    }
}

static inline std::pair<double, double> scanner_offset_to_displacement(const Pose& scanner_offset)
{
    const double d = euclidean_distance({0, 0, 0}, scanner_offset);
    const double theta = std::atan2(scanner_offset.y, scanner_offset.x);

    return {d, theta};
}

Pose MCL::sensor_position(const Pose& frame_position, const Pose& scanner_offset)
{
    const auto [scanner_d, scanner_theta] = scanner_offset_to_displacement(scanner_offset);
    const double dx = std::cos(frame_position.theta + scanner_theta) * scanner_d;
    const double dy = std::sin(frame_position.theta + scanner_theta) * scanner_d;

    Pose ret = frame_position;
    ret.x += dx;
    ret.y += dy;
    ret.theta += scanner_offset.theta;

    return ret;
}

void MCL::update(const std::vector<std::tuple<double, double>>& scan,
                 double stddev,
                 double max_dist,
                 const Pose& scanner_offset,
                 int n_threads)
{
    if (n_threads == -1)
    {
        n_threads = 2 * std::thread::hardware_concurrency();
    }
    else
    {
        n_threads = std::min(static_cast<unsigned>(n_threads), std::thread::hardware_concurrency());
    }
    assert(n_threads);

    const auto [scanner_d, scanner_theta] = scanner_offset_to_displacement(scanner_offset);

    std::vector<std::thread> threads(n_threads);
    const int number_particles_per_thread = m_particles.size() / threads.size();
    for (unsigned i = 0; i < threads.size(); ++i)
    {
        const int start = i * number_particles_per_thread;
        int n = number_particles_per_thread;
        if (i == threads.size() - 1) n += m_particles.size() % threads.size();

        threads[i] = std::thread(
            &MCL::update_inner,
            this,
            scan,
            stddev,
            max_dist,
            start,
            n,
            std::tuple<double, double, double>{scanner_d, scanner_theta, scanner_offset.theta});
    }

    for (std::thread& thread : threads)
    {
        thread.join();
    }

    resample_particles(n_threads);
}

static void probabilistic_fitness_selection_inner(std::vector<Particle>& new_particles,
                                                  int start,
                                                  int n)
{
    for (int i = start; i < start + n; ++i)
    {
        new_particles[i].map = new_particles[i].map.clone();
    }
}

std::vector<Particle> MCL::probabilistic_fitness_selection(int n_threads)
{
    // O(nlog m) where m = particles.size()
    double sum = 0;
    for (Particle& particle : m_particles)
    {
        sum += particle.weight;
    }

    static thread_local std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> distribution(0, sum);

    std::vector<double> cumulative_sums;
    cumulative_sums.reserve(m_particles.size());
    cumulative_sums.push_back(m_particles.front().weight);
    for (unsigned i = 1; i < m_particles.size(); ++i)
    {
        cumulative_sums.push_back(cumulative_sums.back() + m_particles[i].weight);
    }

    std::vector<Particle> new_particles(m_particles.size());
    for (unsigned k = 0; k < m_particles.size(); ++k)
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

        new_particles[k] = m_particles[i];
    }

    std::vector<std::thread> threads(n_threads);
    const int number_particles_per_thread = m_particles.size() / threads.size();
    for (unsigned i = 0; i < threads.size(); ++i)
    {
        const int start = i * number_particles_per_thread;
        int n = number_particles_per_thread;
        if (i == threads.size() - 1) n += m_particles.size() % threads.size();

        threads[i] =
            std::thread(probabilistic_fitness_selection_inner, std::ref(new_particles), start, n);
    }

    for (std::thread& thread : threads)
    {
        thread.join();
    }

    return new_particles;
}

void MCL::resample_particles(int n_threads)
{
    std::vector<Particle> new_particles = probabilistic_fitness_selection(n_threads);
    std::swap(new_particles, m_particles);
    std::sort(m_particles.begin(), m_particles.end(), [](const Particle& lhs, const Particle& rhs) {
        return lhs.weight > rhs.weight;
    });
}

}  // namespace slam
