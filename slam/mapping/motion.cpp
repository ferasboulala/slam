#include "motion.h"
#include "util.h"

#include <cmath>

namespace slam
{
Pose sample_motion_model_odometry(const Odometry& odom, const Pose& pose,
                                  std::array<double, 4> alphas)
{
    const double rotation_1 =
        odom.rotation_1 - sample_normal_distribution(std::sqrt(
                              alphas[0] * std::pow(odom.rotation_1, 2) +
                              alphas[1] * std::pow(odom.translation, 2)));
    const double translation =
        odom.translation - sample_normal_distribution(std::sqrt(
                               alphas[2] * std::pow(odom.translation, 2) +
                               alphas[3] * (std::pow(odom.rotation_1, 2) +
                                            ::pow(odom.rotation_2, 2))));
    const double rotation_2 =
        odom.rotation_2 - sample_normal_distribution(std::sqrt(
                              alphas[0] * std::pow(odom.rotation_2, 2) +
                              alphas[1] * std::pow(odom.translation, 2)));

    const double x = pose.x + translation * std::cos(pose.theta + rotation_1);
    const double y = pose.y + translation * std::sin(pose.theta + rotation_1);
    const double theta = normalize_angle(pose.theta + rotation_1 + rotation_2);

    return {x, y, theta};
}

Pose sample_motion_model_velocity(const Velocity& vel, const Pose& pose,
                                  double dt, std::array<double, 6> alphas)
{
    constexpr double EPSILON = 1e-6;
    const double v = vel.v + sample_normal_distribution(
                                 std::sqrt(alphas[0] * std::pow(vel.v, 2) +
                                           alphas[1] * std::pow(vel.w, 2)));
    const double w = vel.w + sample_normal_distribution(
                                 std::sqrt(alphas[2] * std::pow(vel.w, 2) +
                                           alphas[3] * std::pow(vel.v, 2)));
    const double gamma = sample_normal_distribution(std::sqrt(
        alphas[4] * std::pow(vel.v, 2) + alphas[5] * std::pow(vel.w, 2)));

    const double x =
        pose.x -
        v / (w == 0 ? 1 : w) *
            (std::sin(pose.theta) + std::sin(pose.theta + (w + EPSILON) * dt));
    const double y =
        pose.y +
        v / (w == 0 ? 1 : w) *
            (std::cos(pose.theta) - std::cos(pose.theta + (w + EPSILON) * dt));
    const double theta = normalize_angle(pose.theta + w * dt + gamma * dt);

    return {x, y, theta};
}

}  // namespace slam
