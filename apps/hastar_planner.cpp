#include <cmath>
#include <string>

#include "colors.h"
#include "hastar.h"
#include "opencv2/opencv.hpp"
#include "thirdparty/log.h"

static slam::HybridAStar* finder;
static cv::Mat map;
static cv::Mat color_map;
static slam::Coordinate A{-1, -1};
static slam::Coordinate B{-1, -1};
static std::vector<slam::Coordinate> path;
static bool draw;

#define DEG2RAD(x) (x * M_PI / 180)

static constexpr double VEL = 10;
static constexpr double STEERING_ANGLE = DEG2RAD(40);
static constexpr double DESIRED_DELTA_STEERING_ANGLE = DEG2RAD(10);
static constexpr double VEHICLE_LENGTH =
    VEL * std::tan(STEERING_ANGLE) / DESIRED_DELTA_STEERING_ANGLE;

static constexpr int POINT_SIZE = 7;

void mouse_callback(int event, int x, int y, int, void*)
{
    bool changed = false;
    switch (event)
    {
        case cv::EVENT_LBUTTONDOWN:
            A = slam::Coordinate{y, x};
            changed = true;
            cv::cvtColor(map, color_map, cv::COLOR_GRAY2RGB);
            cv::circle(color_map, {x, y}, POINT_SIZE, {0, 0, 255}, cv::FILLED);
            if (B.i != -1) cv::circle(color_map, {B.j, B.i}, POINT_SIZE, {255, 0, 0}, cv::FILLED);
            break;
        case cv::EVENT_RBUTTONDOWN:
        {
            B = slam::Coordinate{y, x};
            changed = true;
            cv::cvtColor(map, color_map, cv::COLOR_GRAY2RGB);
            if (A.i != -1) cv::circle(color_map, {A.j, A.i}, POINT_SIZE, {0, 0, 255}, cv::FILLED);
            cv::circle(color_map, {x, y}, POINT_SIZE, {255, 0, 0}, cv::FILLED);
            break;
        }
        default:
            break;
    }

    if (changed)
    {
        cv::imshow("hastar", color_map);
    }

    if (changed && A.i != -1 && B.i != -1)
    {
        log_info("computing path for %d %d -> %d %d", A.i, A.j, B.i, B.j);
        const slam::Pose A_pose = slam::image_coordinates_to_pose(map, A);
        slam::Pose B_pose = slam::image_coordinates_to_pose(map, B);
        B_pose.theta = M_PI / 2;
        finder->reset(map, A_pose, B_pose, VEL, STEERING_ANGLE, VEHICLE_LENGTH, 5, 3, 5, true);
        cv::Mat* canvas = draw ? &color_map : nullptr;
        while (!finder->pathfind(canvas))
        {
            if (draw)
            {
                cv::imshow("hastar", *canvas);
                const int key = cv::waitKey(1);
                if (key == 113) exit(0);
            }
        }

        slam::Coordinate prev = A;
        const std::vector<slam::Coordinate> path = finder->recover_path();
        for (const slam::Coordinate& coord : path)
        {
            cv::line(color_map, {prev.j, prev.i}, {coord.j, coord.i}, {0, 255, 0}, 2);
            prev = coord;
        }

        cv::imshow("hastar", color_map);
        if (path.empty())
            log_info("No path found path after %u states explored", finder->size());
        else
            log_info("Found path after %u states explored", finder->size());
    }
}

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        log_error("usage : %s <image> kernel_size (draw|nodraw)", argv[0]);
        return -1;
    }

    const int kernel_size = std::stoi(argv[2]);
    draw = std::string(argv[3]) == "draw";

    map = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    const cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    cv::erode(map, map, kernel);
    cv::threshold(map, map, 128, 255, cv::THRESH_BINARY);

    cv::namedWindow("hastar");
    cv::setMouseCallback("hastar", mouse_callback);

    cv::imshow("hastar", map);
    cv::cvtColor(map, color_map, cv::COLOR_GRAY2RGB);
    finder = new slam::HybridAStar(map, {}, {}, VEL, STEERING_ANGLE, VEHICLE_LENGTH, 5, 3, 5, true);
    while (true)
    {
        const int key = cv::waitKey(0);
        if (key == 113)  // q
            break;
    }

    delete finder;

    return 0;
}
