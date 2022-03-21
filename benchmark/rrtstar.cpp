#include <cmath>

#include <benchmark/benchmark.h>

#include "rrtstar.h"
#include "opencv2/opencv.hpp"

#define DEG2RAD(x) (x * M_PI / 180)

static const char *IMG_FILENAME = "/home/ubuntu/Documents/slam/assets/floor_plan.png";
static constexpr unsigned N_BENCHMARK_ITER = 10;
static constexpr unsigned KERNEL_SIZE = 15;
static constexpr unsigned REACH = 20;
static constexpr unsigned RADIUS = 50;

void benchmark_hastar(benchmark::State &state)
{
    cv::Mat map = cv::imread(IMG_FILENAME, cv::IMREAD_GRAYSCALE);
    cv::threshold(map, map, 128, 1.0, cv::THRESH_BINARY);
    map.convertTo(map, CV_64F);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(KERNEL_SIZE, KERNEL_SIZE));
    cv::erode(map, map, kernel);

    const slam::Coordinate A{150, 450};
    const slam::Coordinate B{450, 750};

    for (auto _ : state) {
        auto finder = slam::RRTStar(map, A, B, 20, 50, 1234);
        while (!finder.pathfind(nullptr)) {}
    }
}

BENCHMARK(benchmark_hastar)->Iterations(N_BENCHMARK_ITER);

BENCHMARK_MAIN();