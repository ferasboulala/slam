#include <cmath>

#include <benchmark/benchmark.h>

#include "hastar.h"
#include "opencv2/opencv.hpp"

#define DEG2RAD(x) (x * M_PI / 180)

static const char *IMG_FILENAME = "assets/floor_plan.png";
static constexpr unsigned N_BENCHMARK_ITER = 10;
static constexpr unsigned KERNEL_SIZE = 15;
static constexpr double VEL = 10;
static constexpr double STEERING_ANGLE = DEG2RAD(40);
static constexpr double DESIRED_DELTA_STEERING_ANGLE = DEG2RAD(10);
static constexpr double VEHICLE_LENGTH = VEL * std::tan(STEERING_ANGLE) / DESIRED_DELTA_STEERING_ANGLE;

void benchmark_hastar(benchmark::State &state)
{
    cv::Mat map = cv::imread(IMG_FILENAME, cv::IMREAD_GRAYSCALE);
    cv::threshold(map, map, 128, 1.0, cv::THRESH_BINARY);
    map.convertTo(map, CV_64F);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(KERNEL_SIZE, KERNEL_SIZE));
    cv::erode(map, map, kernel);

    const slam::Pose A{400, 450, 0};
    const slam::Pose B{700, 150, 0};

    for (auto _ : state) {
        auto finder = slam::HybridAStar(map, A, B, VEL, STEERING_ANGLE, VEHICLE_LENGTH, 5, 3, 5, true);
        while (!finder.pathfind(nullptr)) {}
    }
}

BENCHMARK(benchmark_hastar)->Iterations(N_BENCHMARK_ITER);

BENCHMARK_MAIN();