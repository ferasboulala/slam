#include "astar.h"
#include "colors.h"
#include "thirdparty/log.h"

#include <string>

#include "opencv2/opencv.hpp"

static cv::Mat map;
static cv::Mat map_;
static cv::Mat color_map;
static slam::Coordinate A{-1, -1};
static slam::Coordinate B{-1, -1};
static std::vector<slam::Coordinate> path;

static constexpr int POINT_SIZE = 10;

#define DRAW

void mouse_callback(int event, int x, int y, int, void *)
{
    bool changed = false;
    switch (event)
    {
        case cv::EVENT_LBUTTONDOWN:
            A = slam::Coordinate{y, x};
            changed = true;
            cv::cvtColor(map_, color_map, cv::COLOR_GRAY2RGB);
            cv::circle(color_map, {x, y}, POINT_SIZE, RED, cv::FILLED);
            if (B.i != -1)
                cv::circle(color_map, {B.j, B.i}, POINT_SIZE, CYAN, cv::FILLED);
            break;
        case cv::EVENT_RBUTTONDOWN:
        {
            B = slam::Coordinate{y, x};
            changed = true;
            cv::cvtColor(map_, color_map, cv::COLOR_GRAY2RGB);
            if (A.i != -1)
                cv::circle(color_map, {A.j, A.i}, POINT_SIZE, RED, cv::FILLED);
            cv::circle(color_map, {x, y}, POINT_SIZE, CYAN, cv::FILLED);
            break;
        }
        default:
            break;
    }

    if (changed && A.i != -1 && B.i != -1)
    {
        log_info("computing path for %d %d -> %d %d", A.i, A.j, B.i, B.j);
        auto finder = slam::AStar(map, A, B);
#ifdef DRAW
        cv::Mat *canvas = &color_map;
#else
        cv::Mat *canvas = nullptr;
#endif
        while (!finder.pathfind(canvas))
        {
#ifdef DRAW
            cv::imshow("dilate", *canvas);
            const int key = cv::waitKey(1);
            if (key == 113) exit(0);
#endif
        }

        for (const slam::Coordinate &coord : finder.recover_path())
        {
            cv::circle(color_map, {coord.j, coord.i}, 1, GREEN, cv::FILLED);
        }
        cv::imshow("dilate", color_map);
    }
    else if (changed)
    {
        cv::imshow("dilate", color_map);
    }
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        log_error("usage : %s <image> kernel_size", argv[0]);
        return -1;
    }

    const int kernel_size = std::stoi(argv[2]);

    map = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::threshold(map, map, 128, 1.0, cv::THRESH_BINARY);
    map.convertTo(map, CV_64F);
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::erode(map, map, kernel);

    cv::namedWindow("dilate");
    cv::setMouseCallback("dilate", mouse_callback);

    cv::imshow("dilate", map);
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
