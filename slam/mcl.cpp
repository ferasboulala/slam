#include "mcl.h"
#include "motion.h"
#include "raycast.h"
#include "util.h"

#include <algorithm>
#include <chrono>
#include <random>
#include <cmath>

constexpr int CANVAS_SIZE = 1500;
constexpr double OCCUPIED_THRESH = 0.5;

namespace slam
{
MCL::MCL(int n_particles, const std::array<double, 4> alphas, double alpha_fast,
         double alpha_slow)
    : m_alphas(alphas),
      m_omega_fast(1.0 / n_particles),
      m_omega_slow(1.0 / n_particles),
      m_alpha_fast(alpha_fast),
      m_alpha_slow(alpha_slow)
{
    m_occupied = cv::Mat(cv::Size(CANVAS_SIZE, CANVAS_SIZE), CV_32S, cv::Scalar(0));
    m_free = cv::Mat(cv::Size(CANVAS_SIZE, CANVAS_SIZE), CV_32S, cv::Scalar(0));

    this->occupancy_grid = cv::Mat(cv::Size(CANVAS_SIZE, CANVAS_SIZE), CV_64F, cv::Scalar(0));
    this->particles.resize(n_particles);
    reset_particles();
}

// TODO : Could be moved to utils
// static Pose random_pose(const Eigen::MatrixXi& map)
// {
//     const size_t X_MAX = map.cols();
//     const size_t Y_MAX = map.rows();

//     thread_local std::mt19937 generator(std::random_device{}());
//     std::uniform_real_distribution<double> x_distribution(0, X_MAX - 1);
//     std::uniform_real_distribution<double> y_distribution(0, Y_MAX - 1);
//     std::uniform_real_distribution<double> theta_distribution(0, 2 * M_PI);

//     Pose pose;
//     pose.x = x_distribution(generator);
//     pose.y = y_distribution(generator);
//     pose.theta = theta_distribution(generator);

//     return pose;
// }

void MCL::reset_particles()
{
    const auto blank_canvas = cv::Mat(cv::Size(CANVAS_SIZE, CANVAS_SIZE), CV_32S, cv::Scalar(-1));
    const double uniform_weight = 1.0 / this->particles.size();
    const Pose center{ 0.5 * CANVAS_SIZE, 0.5 * CANVAS_SIZE, M_PI / 2};
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
    m_bounding_box = Box{ CANVAS_SIZE - 1, CANVAS_SIZE - 1, 0, 0 };
    const double range = lidar.stop - lidar.start;
    const double step = range / lidar.n_rays;
    for (Particle& particle : this->particles)
    {
        double weight = 0;
        const Pose pose = particle.pose;
        particle.pose.theta -= range / 2;
        for (int i = 0; i < lidar.n_rays; ++i)
        {
            double w = measurement_model_beam(scans[i], lidar.stddev, particle, m_occupied, m_free,
                                             m_bounding_box, lidar.max_dist);
            w = std::max(0.99, w);
            weight += std::log(w / (1 - w));
            particle.pose.theta += step;
        }
        particle.weight = 1 - (1 / (1 + weight)); // More numerically stable
        particle.pose = pose;
    }

    map();
    // filter_particles();
    resample_particles();
}

std::vector<Particle> MCL::fitness_selection(int n)
{
    // O(n + m log m)
    std::sort(this->particles.begin(), this->particles.end(),
        [](const Particle &lhs, const Particle &rhs) { return lhs.weight > rhs.weight; });

    static thread_local std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<int> distribution(1, this->particles.size() * (this->particles.size() + 1) / 2);

    std::vector<Particle> new_particles;
    new_particles.reserve(n);
    for (int i = 0; i < n; ++i)
    {
        const int cutoff = distribution(generator);
        const int idx = this->particles.size() - (-1 + std::sqrt(1 + 8 * cutoff)) / 2 - 1;
        new_particles.push_back(this->particles[idx]);
    }

    return new_particles;
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
        cumulative_sums.push_back(cumulative_sums.back() + this->particles[i].weight);
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
            if (cumulative_sums[mid] == cutoff) {
                i = mid;
                j = mid;
            } else if (cumulative_sums[mid] < cutoff) {
                i = mid + 1;
            } else {
                j = mid;
            }
        }

        new_particles.push_back(this->particles[i]);
    }

    return new_particles;
}

// void MCL::filter_particles()
// {
//     std::vector<Particle> filtered_particles;
//     filtered_particles.reserve(this->particles.size());
//     for (const Particle &particle : this->particles)
//     {
//         const auto coord = pose_to_image_coordinates(particle.map, particle.pose);
//         int i, j;
//         std::tie(i, j) = coord;
//         if (!within_boundaries(particle.map, i, j) ||
//             particle.map(i, j) > OCCUPIED_THRESH)
//         {
//             filtered_particles.push_back({random_pose(particle.map), particle.weight});
//         }
//         else
//         {
//             filtered_particles.push_back(particle);
//         }
//     }

//     std::swap(filtered_particles, this->particles);
// }

int MCL::compute_number_new_particles()
{
    constexpr double RATIO_NEW_PARTICLES = 0.05;
    return this->particles.size() * RATIO_NEW_PARTICLES;
}

void MCL::resample_particles()
{
    // const int number_of_new_particles = compute_number_new_particles(); 
    std::vector<Particle> new_particles = probabilistic_fitness_selection(
        this->particles.size()); // - number_of_new_particles);
    // for (int i = 0; i < number_of_new_particles; ++i)
    // {
    //     new_particles.push_back(
    //         {random_pose(new_particles.back().map), this->particles.back().weight});
    // }

    std::swap(new_particles, this->particles);
}

void MCL::map()
{
    for (int i = m_bounding_box.start_i; i < m_bounding_box.stop_i + 1; ++i)
    {
        for (int j = m_bounding_box.start_j; j < m_bounding_box.stop_j + 1; ++j)
        {
            const int a = m_occupied.at<int>(i, j);
            const int b = m_free.at<int>(i, j);
            this->occupancy_grid.at<double>(i, j) = (static_cast<double>(a + 1) / (a + b + 2));
        }
    }
}

}  // namespace slam
