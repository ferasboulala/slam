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
    Q = 113,
    U = 117
};

void draw_particle(
    cv::Mat& img, const slam::Pose& pose, cv::Scalar color, int size, bool filled = false)
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
    if (argc < 5)
    {
        log_error("usage : %s map.png n_particles canvas_size hz", argv[0]);
        return -1;
    }

    const bool debug = argc > 5;

    cv::Mat map = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    const unsigned n_particles = atoi(argv[2]);
    const unsigned canvas_size = atoi(argv[3]);
    const unsigned hz = atoi(argv[4]);

    // Binary image
    cv::threshold(map, map, 128, 1.0, cv::THRESH_BINARY);
    map.convertTo(map, CV_32S);

    constexpr double FAKE_LIDAR_STDDEV = 5;
    constexpr double FAKE_LIDAR_MAX_DIST = 500;
    constexpr double FAKE_LIDAR_START_ANGLE = 0;
    constexpr double FAKE_LIDAR_STOP_ANGLE = 2 * M_PI;
    constexpr unsigned N_RAYS = 90;
    slam::FakeLidar fake_lidar(FAKE_LIDAR_START_ANGLE,
                               FAKE_LIDAR_STOP_ANGLE,
                               FAKE_LIDAR_MAX_DIST,
                               FAKE_LIDAR_STDDEV,
                               N_RAYS);

    constexpr slam::Pose SCANNER_OFFSET = {0, 30, 0};
    slam::MCL mcl(n_particles, {canvas_size, canvas_size});

    slam::Pose real_position{400, 400, M_PI / 90};

    // This is the image that is displayed every frame
    cv::Mat map_image_frame;
    const int WAIT_TIME = debug ? 0 : 25;
    const int EVERY_OTHER = debug ? 1 : std::min(25.f, 1000.f / hz / 25);
    int frame = 0;

    while (true)
    {
        map_image_frame = mcl.get_particles().front().map.clone();
        cv::cvtColor(map_image_frame, map_image_frame, cv::COLOR_GRAY2RGB);
        for (const slam::Particle& particle : mcl.get_particles())
        {
            draw_particle(map_image_frame, particle.pose, {0, 255, 0}, 5, true);
        }

        const slam::Pose& average_pose = slam::average_pose(mcl.get_particles());
        draw_particle(map_image_frame, average_pose, {0, 0, 255}, 5, true);
        draw_particle(map_image_frame,
                      slam::MCL::sensor_position(average_pose, SCANNER_OFFSET),
                      {0, 0, 255},
                      3,
                      true);

        cv::resize(map_image_frame, map_image_frame, cv::Size(1000, 1000), cv::INTER_LINEAR);
        cv::imshow("slam", map_image_frame);

        const int key = cv::waitKey(WAIT_TIME);
        const slam::Odometry odom = getUserInput(key);
        mcl.predict(odom, {0.001, 0.001, 0.001, 0.001});

        real_position =
            slam::sample_motion_model_odometry(odom, real_position, {0.001, 0.001, 0.001, 0.001});

        if (debug && static_cast<Key>(cv::waitKey(WAIT_TIME)) != Key::U)
        {
            continue;
        }

        if (++frame % EVERY_OTHER == 0)
        {
            const slam::Pose sensor_position =
                slam::MCL::sensor_position(real_position, SCANNER_OFFSET);
            const std::vector<slam::Pose> hits = fake_lidar.scan(map, sensor_position);
            std::vector<std::tuple<double, double>> scans;
            scans.reserve(hits.size());
            const double range = fake_lidar.stop - fake_lidar.start;
            const double step = range / fake_lidar.n_rays;
            int i = 0;
            for (const slam::Pose& hit : hits)
            {
                double dist;
                if (hit.x != -1)
                {
                    dist = std::sqrt(std::pow(hit.y - sensor_position.y, 2) +
                                     std::pow(hit.x - sensor_position.x, 2));
                }
                else
                {
                    dist = fake_lidar.max_dist;
                }
                const double angle = i++ * step - range / 2;
                scans.push_back({angle, dist});
            }

            mcl.update(scans, FAKE_LIDAR_STDDEV, fake_lidar.max_dist, SCANNER_OFFSET);
        }
    }

    return 0;
}
