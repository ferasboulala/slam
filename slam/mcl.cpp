#include "mcl.h"
#include "motion.h"
#include "raycast.h"
#include "util.h"

#include <algorithm>
#include <chrono>
#include <random>

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
    const double uniform_weight = 1.0 / this->particles.size();
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
    constexpr std::array<double, 4> alphas = {1, 1, 1, 1};
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
        double weight = 1;
        Pose pose = particle.pose;
        pose.theta -= range / 2;
        for (int i = 0; i < lidar.n_rays; ++i)
        {
            weight *= measurement_model_beam(scans[i], lidar.stddev, map, pose,
                                             lidar.max_dist);
            pose.theta += step;
        }
        particle.weight = std::max(1, weight);
    }

    resample_particles(map);
}

std::vector<int> np_random_choices_with_weights(std::vector<double> weights,
                                                int n)
{
    static thread_local std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> distribution(0, 1);

    std::vector<int> weight_indices;
    weight_indices.reserve(weights.size());
    for (unsigned i = 0; i < weights.size(); ++i)
    {
        weight_indices.push_back(i);
    }
    std::vector<int> indices;
    indices.reserve(n);
    // O(n * m) where m = weights.size()
    for (int i = 0; i < n; ++i)
    {
        const double total = std::accumulate(weights.begin(), weights.end(), 0);
        const double cutoff = distribution(generator);
        double sum = 0;
        for (unsigned j = 0; j < weights.size(); ++j)
        {
            sum += weights[j] / total;
            if (sum >= cutoff || j == weights.size() - 1)
            {
                indices.push_back(weight_indices[j]);
                std::swap(weights.back(), weights[j]);
                std::swap(weight_indices.back(), weight_indices[j]);
                weights.pop_back();
                weight_indices.pop_back();
                break;
            }
        }
    }

    return indices;
}

void MCL::resample_particles(const Eigen::MatrixXf& map)
{
    constexpr double RATIO_NEW_PARTICLES = 0.1;
    double weight_total = 0;
    for (const Particle& particle : this->particles)
    {
        weight_total += particle.weight;
    }
    const double average_weight = weight_total / this->particles.size();
    m_omega_slow += m_alpha_slow * (average_weight - m_omega_slow);
    m_omega_fast += m_alpha_fast * (average_weight - m_omega_fast);
    const double ratio = std::max(
        RATIO_NEW_PARTICLES, std::max(0.0, 1.0 - m_omega_fast / m_omega_slow));
    const int number_of_new_particles = ratio * this->particles.size();

    std::vector<double> weights;
    weights.reserve(this->particles.size());
    for (const Particle& particle : this->particles)
    {
        weights.push_back(particle.weight);
    }

    const std::vector<int> indices = np_random_choices_with_weights(
        weights, this->particles.size() - number_of_new_particles);

    std::vector<Particle> new_particles;
    new_particles.reserve(this->particles.size());
    for (int i : indices)
    {
        new_particles.push_back(this->particles[i]);
    }

    for (int i = 0; i < number_of_new_particles; ++i)
    {
        new_particles.push_back({random_pose(map), 0});
    }

    std::swap(new_particles, this->particles);
}

}  // namespace slam
