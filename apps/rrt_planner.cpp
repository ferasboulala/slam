#include <string>

#include "colors.h"
#include "opencv2/opencv.hpp"
#include "rrtstar.h"
#include "thirdparty/log.h"

static cv::Mat map;
static cv::Mat map_;
static cv::Mat color_map;
static slam::Coordinate A{-1, -1};
static slam::Coordinate B{-1, -1};
static std::vector<slam::Coordinate> path;
static unsigned n_points;
static bool draw;

static constexpr int POINT_SIZE = 10;

void mouse_callback(int event, int x, int y, int, void*)
{
    bool changed = false;
    switch (event)
    {
        case cv::EVENT_LBUTTONDOWN:
            A = slam::Coordinate{y, x};
            changed = true;
            cv::cvtColor(map_, color_map, cv::COLOR_GRAY2RGB);
            cv::circle(color_map, {x, y}, POINT_SIZE, RED, cv::FILLED);
            if (B.i != -1) cv::circle(color_map, {B.j, B.i}, POINT_SIZE, CYAN, cv::FILLED);
            break;
        case cv::EVENT_RBUTTONDOWN:
        {
            B = slam::Coordinate{y, x};
            changed = true;
            cv::cvtColor(map_, color_map, cv::COLOR_GRAY2RGB);
            if (A.i != -1) cv::circle(color_map, {A.j, A.i}, POINT_SIZE, RED, cv::FILLED);
            cv::circle(color_map, {x, y}, POINT_SIZE, CYAN, cv::FILLED);
            break;
        }
        default:
            break;
    }

    if (changed && A.i != -1 && B.i != -1)
    {
        log_info("computing path for %d %d -> %d %d", A.i, A.j, B.i, B.j);
        auto finder = slam::RRTStar(map, A, B, 20, 50);

        cv::Mat* canvas = draw ? &color_map : nullptr;
        while (!finder.pathfind(canvas) || finder.size() < n_points)
        {
            if (draw)
            {
                cv::imshow("rrt", *canvas);
                const int key = cv::waitKey(1);
                if (key == 113) exit(0);
            }
        }

        slam::Coordinate prev = B;
        for (const slam::Coordinate& coord : finder.recover_path())
        {
            cv::line(color_map, {prev.j, prev.i}, {coord.j, coord.i}, GREEN, 3);
            prev = coord;
        }
        cv::imshow("rrt", color_map);
        log_info("Found path after %u states explored", finder.size());
    }
    else if (changed)
    {
        cv::imshow("rrt", color_map);
    }
}

int main(int argc, char** argv)
{
    if (argc != 5)
    {
        log_error("usage : %s <image> kernel_size n_points (draw|nodraw)", argv[0]);
        return -1;
    }

    const int kernel_size = std::stoi(argv[2]);
    n_points = std::stoi(argv[3]);
    draw = std::string(argv[4]) == "draw";

    map = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::threshold(map, map, 128, 1.0, cv::THRESH_BINARY);
    map.convertTo(map, CV_64F);
    const cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
    cv::erode(map, map, kernel);

    cv::namedWindow("rrt");
    cv::setMouseCallback("rrt", mouse_callback);

    cv::imshow("rrt", map);
    map.convertTo(map_, CV_32F);
    cv::cvtColor(map_, color_map, cv::COLOR_GRAY2RGB);
    while (true)
    {
        const int key = cv::waitKey(0);
        if (key == 113)  // q
            break;
    }

    return 0;
}
