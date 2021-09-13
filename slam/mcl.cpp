#include "mcl.h"
#include "motion.h"

#include <chrono>
#include <random>

namespace slam
{

MCL::MCL(const Eigen::MatrixXf& map, int n_particles, double alpha_fast,
         double alpha_slow)
  : m_map(map)
  , m_omega_fast(1.0 / n_particles)
  , m_omega_slow(1.0 / n_particles)
  , m_alpha_fast(alpha_fast)
  , m_alpha_slow(alpha_slow)
{
    this->particles.resize(n_particles);
    reset_particles();
}

static Pose random_pose(const Eigen::MatrixXf& map)
{
    const size_t X_MAX = map.cols();
    const size_t Y_MAX = map.rows();

    std::default_random_engine generator;
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
    const double uniform_weight = 1 / this->particles.size();
    for (Particle &particle : this->particles)\
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

    return { avg_x, avg_y, avg_theta };
}

void MCL::predict(const Odometry &odom)
{
    // TODO : Either make it a parameter or create an odom objet that describes it
    constexpr std::array<double, 4> alphas = { 0.01 };
    for (Particle &particle : this->particles)
    {
        particle.pose = sample_motion_model_odometry(odom, particle.pose, alphas);
    }
}

void MCL::update()
{

}

} // namespace slam
