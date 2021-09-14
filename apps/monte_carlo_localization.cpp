#include "mcl.h"
#include "lidar.h"
#include "util.h"
#include "thirdparty/log.h"

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

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

    slam::Lidar lidar(0, M_PI, 500, 100, 1);

    slam::MCL mcl(map, 1000);
    const slam::Pose estimate_pose = mcl.average_pose();
    const auto image_estimate_coordinates = slam::pose_to_image_coordinates(map, estimate_pose);
    cv::circle(map_image, { std::get<0>(image_estimate_coordinates), std::get<1>(image_estimate_coordinates)}, 10, {255});

    // TODO : Create drawing functions for particles
    log_info("Estimated position = (%f, %f, %f)", estimate_pose.x, estimate_pose.y, estimate_pose.theta);
    for (const slam::Particle &particle : mcl.particles)
    {
        const auto coord = slam::pose_to_image_coordinates(map, particle.pose);
        cv::circle(map_image, { std::get<0>(coord), std::get<1>(coord) }, 3, {128}, CV_FILLED);
    }

    cv::imshow("Raycast", (1 - map_image) * 255);
    cv::waitKey(0);

    return 0;
}
