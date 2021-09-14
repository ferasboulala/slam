#include "lidar.h"
#include "mcl.h"
#include "motion.h"
#include "pose.h"
#include "thirdparty/log.h"
#include "util.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>

constexpr int UP = 1113938;
constexpr int LEFT = 1113937;
constexpr int DOWN = 1113940;
constexpr int RIGHT = 1113939;
constexpr int Q = 1048689;

constexpr double VEL = 1;
constexpr double ANG = 0.01;

void draw_particle(cv::Mat& img, const Eigen::MatrixXf& map,
                   const slam::Pose& pose, const cv::Scalar& color, int size,
                   bool filled = false)
{
    const auto coord = slam::pose_to_image_coordinates(map, pose);
    cv::circle(img, {std::get<1>(coord), std::get<0>(coord)}, size, color,
               filled ? CV_FILLED : 0);
}

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        log_error("usage : %s map.png", argv[0]);
        return -1;
    }

    // TODO : Make this thresholding and eigen2cv process a function
    cv::Mat map_image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::threshold(map_image, map_image, 128, 1.0, cv::THRESH_BINARY_INV);

    Eigen::MatrixXf map(map_image.rows, map_image.cols);
    cv::cv2eigen(map_image, map);
    cv::cvtColor(map_image, map_image, cv::COLOR_GRAY2BGR);

    slam::Lidar lidar(0, M_PI, 500, 0.1, 20);
    slam::MCL mcl(map, 500);

    slam::Pose real_position{map.cols() / 2.0 + 200, map.rows() / 2.0,
                             M_PI / 2};
    cv::Mat map_image_frame;
    map_image.copyTo(map_image_frame);
    while (true)
    {
        const slam::Pose estimate_pose = mcl.average_pose();
        draw_particle(map_image_frame, map, real_position, {0, 0, 255}, 15,
                      true);
        draw_particle(map_image_frame, map, estimate_pose, {0, 255, 0}, 10,
                      true);

        for (const slam::Particle& particle : mcl.particles)
        {
            draw_particle(map_image_frame, map, particle.pose, {255, 128, 0}, 2,
                          CV_FILLED);
        }
        cv::imshow("Raycast", map_image_frame * 255);
        map_image.copyTo(map_image_frame);

        const int key = cv::waitKey(0);
        slam::Odometry odom{0, 0, 0};
        switch (key)
        {
            case Q:
                return 0;
            case UP:
                odom.translation = VEL;
                break;
            case DOWN:
                odom.translation = -VEL;
                break;
            case LEFT:
                odom.rotation_1 = ANG / 2;
                odom.rotation_2 = ANG / 2;
                break;
            case RIGHT:
                odom.rotation_1 = -ANG / 2;
                odom.rotation_2 = -ANG / 2;
                break;
            default:
                break;
        }

        mcl.predict(odom);

        real_position = slam::sample_motion_model_odometry(odom, real_position);
        const auto real_position_coord =
            slam::pose_to_image_coordinates(map, real_position);

        // make this a function
        const std::vector<slam::Pose> hits = lidar.scan(map, real_position);
        std::vector<double> distances;
        distances.reserve(hits.size());
        for (const slam::Pose& hit : hits)
        {
            double dist;
            if (hit.x != -1)
            {
                const auto coord = slam::pose_to_image_coordinates(map, hit);
                cv::line(map_image_frame,
                         {std::get<1>(coord), std::get<0>(coord)},
                         {std::get<1>(real_position_coord),
                          std::get<0>(real_position_coord)},
                         {255, 255, 255});
                dist = lidar.max_dist;
            }
            else
            {
                dist = std::sqrt(std::pow(hit.y - real_position.y, 2) +
                                 std::pow(hit.x - real_position.x, 2));
            }
            distances.push_back(dist);
        }

        mcl.update(lidar, distances, map);
    }

    return 0;
}
