#include "raycast.h"
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

    cv::Mat map_image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::threshold(map_image, map_image, 128, 1.0, cv::THRESH_BINARY_INV);

    Eigen::MatrixXf map(map_image.rows, map_image.cols);
    cv::cv2eigen(map_image, map);

    constexpr int N_RAYS = 10;
    constexpr double RANGE = M_PI;
    constexpr double STEP = RANGE / N_RAYS;
    const double Z_MAX = std::sqrt(std::pow(map.cols(), 2) + std::pow(map.rows(), 2));
    slam::Pose pose{map.cols() / 2.0, map.rows() / 2.0, 0};
    for (int i = 0; i < N_RAYS; ++i)
    {
        pose.theta = STEP * i;
        const slam::Pose hit = slam::raycast(map, pose, Z_MAX);
        if (hit.theta)
            cv::line(map_image, { (int)hit.x, (int)hit.y }, { (int)pose.x, (int)pose.y }, { 255, 0, 0});
    }

    cv::imshow("Raycast", (1 - map_image) * 255);
    cv::waitKey(0);

    return 0;
}
