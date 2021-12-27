#include <cmath>
#include <opencv2/opencv.hpp>

#include "colors.h"
#include "fake_lidar.h"
#include "mcl.h"
#include "motion.h"
#include "pose.h"
#include "thirdparty/log.h"
#include "util.h"

enum class Key : int
{
    UP = 82,
    LEFT = 81,
    DOWN = 84,
    RIGHT = 83,
    Q = 113
};

void draw_particle(cv::Mat& img, const slam::Pose& pose, cv::Scalar color, int size, bool filled = false)
{
    const auto coord = slam::pose_to_image_coordinates(img, pose);
    int i, j;
    std::tie(i, j) = coord;
    cv::circle(img, {j, i}, size, color, filled ? cv::FILLED : 0);

    const double x = pose.x + 10 * size * std::cos(pose.theta);
    const double y = pose.y + 10 * size * std::sin(pose.theta);
    const auto coord_ = slam::pose_to_image_coordinates(img, {x, y, 0});
    int i_, j_;
    std::tie(i_, j_) = coord_;
    cv::line(img, {j, i}, {j_, i_}, color);
}

slam::Odometry getUserInput(int key)
{
    constexpr double VEL = 2.5;
    constexpr double ANG = 0.05;

    slam::Odometry odom{0, 0, 0};
    switch (static_cast<Key>(key))
    {
        case Key::Q:
            exit(0);
        case Key::UP:
            odom.translation = VEL;
            break;
        case Key::DOWN:
            odom.translation = -VEL;
            break;
        case Key::LEFT:
            odom.rotation_1 = ANG / 2;
            odom.rotation_2 = ANG / 2;
            break;
        case Key::RIGHT:
            odom.rotation_1 = -ANG / 2;
            odom.rotation_2 = -ANG / 2;
            break;
        default:
            break;
    }

    return odom;
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        log_error("usage : %s map.png", argv[0]);
        return -1;
    }

    cv::Mat map = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    // Ends up as a binary image
    cv::threshold(map, map, 128, 1.0, cv::THRESH_BINARY);
    map.convertTo(map, CV_32S);

    slam::FakeLidar fake_lidar(0, 2 * M_PI, 500, 1, 90);
    slam::MCL mcl(25);
    slam::Pose real_position{400, 400, M_PI};

    // This is the image that is displayed every frame
    cv::Mat map_image_frame;
    constexpr int WAIT_TIME = 25;
    constexpr int EVERY_OTHER = 200 / WAIT_TIME;
    int frame = 0;
    while (true)
    {
        map_image_frame = mcl.get_particles().front().map.clone();
        map_image_frame.convertTo(map_image_frame, CV_32FC3);
        cv::cvtColor(map_image_frame, map_image_frame, cv::COLOR_GRAY2RGB);
        for (const slam::Particle& particle : mcl.get_particles())
        {
            draw_particle(map_image_frame, particle.pose, GREEN, 5, true);
        }
        cv::imshow("slam", map_image_frame);

        const int key = cv::waitKey(WAIT_TIME);
        const slam::Odometry odom = getUserInput(key);
        mcl.predict(odom, {0.0005, 0.0005, 0.01, 0.01});

        // Use 0s for alphas meaning no error in the motion
        real_position = slam::sample_motion_model_odometry(odom, real_position);

        // TODO: make this a function
        if (++frame % EVERY_OTHER == 0)
        {
            const std::vector<slam::Pose> hits = fake_lidar.scan(map, real_position);
            std::vector<std::tuple<double, double>> scans;
            scans.reserve(hits.size());
            constexpr double range = 2 * M_PI;
            const unsigned n_rays = 90;
            constexpr double step = range / n_rays;
            int i = 0;
            for (const slam::Pose& hit : hits)
            {
                double dist;
                if (hit.x != -1)
                {
                    dist = std::sqrt(std::pow(hit.y - real_position.y, 2) + std::pow(hit.x - real_position.x, 2));
                }
                else
                {
                    dist = fake_lidar.max_dist;
                }
                const double angle = i++ * step - range / 2;
                scans.push_back({angle, dist});
            }

            constexpr double stddev = 1;
            constexpr double max_dist = 500;
            mcl.update(scans, stddev, max_dist);
        }
    }

    return 0;
}
