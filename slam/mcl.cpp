#include "mcl.h"
#include "motion.h"
#include "raycast.h"
#include "util.h"

#include <algorithm>
#include <chrono>
#include <random>
#include <cmath>

namespace slam
{
MCL::MCL(const Eigen::MatrixXf& map, int n_particles, double alpha_fast,
         double alpha_slow)
    : m_map(map),
      m_omega_fast(1.0 / n_particles),
      m_omega_slow(1.0 / n_particles),
      m_alpha_fast(alpha_fast),
      m_alpha_slow(alpha_slow)
{
    this->particles.resize(n_particles);
    reset_particles();
}

static Pose random_pose(const Eigen::MatrixXf& map)
{
    const size_t X_MAX = map.cols();
    const size_t Y_MAX = map.rows();

    static thread_local std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> x_distribution(0, X_MAX - 1);
    std::uniform_real_distribution<double> y_distribution(0, Y_MAX - 1);
    std::uniform_real_distribution<double> theta_distribution(0, 2 * M_PI);

    Pose pose;
    pose.x = x_distribution(generator);
    pose.y = y_distribution(generator);
    pose.theta = theta_distribution(generator);

    return pose;
}

void MCL::reset_particles()
{
    const double uniform_weight = std::log(1.0 / this->particles.size());
    for (Particle& particle : this->particles)
    {
        particle.weight = uniform_weight;
        particle.pose = random_pose(m_map);
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
    // TODO : Either make it a parameter or create an odom objet that describes
    // it
    constexpr std::array<double, 4> alphas = {0.01, 0.01, 0.1, 0.1};
    for (Particle& particle : this->particles)
    {
        particle.pose =
            sample_motion_model_odometry(odom, particle.pose, alphas);
    }
}

void MCL::update(const Lidar& lidar, const std::vector<double>& scans,
                 const Eigen::MatrixXf& map)
{
    const double range = lidar.stop - lidar.start;
    const double step = range / lidar.n_rays;
    for (Particle& particle : this->particles)
    {
        double weight = 0;
        Pose pose = particle.pose;
        pose.theta -= range / 2;
        for (int i = 0; i < lidar.n_rays; ++i)
        {
            weight += std::log(measurement_model_beam(scans[i], lidar.stddev, map, pose,
                                             lidar.max_dist));
            pose.theta += step;
        }
        particle.weight = weight;
    }

    filter_particles(map);
    resample_particles(map);
}

std::vector<Particle> fitness_selection(const std::vector<Particle> &particles,
                                                int n)
{
    static thread_local std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int> distribution(1, particles.size() * (particles.size() + 1) / 2);

    std::vector<Particle> new_particles;
    new_particles.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        const int cutoff = distribution(generator);
        const int idx = particles.size() - (-1 + std::sqrt(1 + 8 * cutoff)) / 2 - 1;
        new_particles.push_back(particles[idx]);
    }

    return new_particles;
}

void MCL::filter_particles(const Eigen::MatrixXf& map)
{
    std::vector<Particle> filtered_particles;
    filtered_particles.reserve(this->particles.size());
    for (const Particle &particle : this->particles)
    {
        const auto coord = pose_to_image_coordinates(map, particle.pose);
        int i, j;
        std::tie(i, j) = coord;
        if (!within_boundaries(map, i, j) || map(i, j))
        {
            filtered_particles.push_back({random_pose(map), 0});
        }
        else
        {
            filtered_particles.push_back(particle);
        }
    }

    std::swap(filtered_particles, this->particles);
}

void MCL::resample_particles(const Eigen::MatrixXf& map)
{
    const int number_of_new_particles = 0.05 * this->particles.size();
    if (!number_of_new_particles)
        return;

    std::sort(this->particles.begin(), this->particles.end(),
        [](const Particle &lhs, const Particle &rhs) { return lhs.weight > rhs.weight; });

    std::vector<Particle> new_particles = fitness_selection(
        this->particles, this->particles.size() - number_of_new_particles);

    for (int i = 0; i < number_of_new_particles; ++i)
    {
        new_particles.push_back({random_pose(map), (this->particles.size() + 1) / 2.0});
    }

    std::swap(new_particles, this->particles);
}

}  // namespace slam
