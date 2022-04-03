#include <cmath>
#include <opencv2/opencv.hpp>
#include <benchmark/benchmark.h>

#include "fake_lidar.h"
#include "mcl.h"
#include "motion.h"
#include "pose.h"
#include "util.h"

static const char *IMG_FILENAME = "/home/ubuntu/Documents/slam/assets/floor_plan.png";
static constexpr unsigned N_BENCHMARK_ITER = 1000;

void benchmark_mcl(benchmark::State &state)
{
    // setup
    cv::Mat map = cv::imread(IMG_FILENAME, cv::IMREAD_GRAYSCALE);
    cv::threshold(map, map, 128, 1.0, cv::THRESH_BINARY);
    map.convertTo(map, CV_32S);

    constexpr double FAKE_LIDAR_STDDEV = 5;
    constexpr double FAKE_LIDAR_MAX_DIST = 500;
    constexpr double FAKE_LIDAR_START_ANGLE = 0;
    constexpr double FAKE_LIDAR_STOP_ANGLE = M_PI;
    slam::FakeLidar fake_lidar(
        FAKE_LIDAR_START_ANGLE, FAKE_LIDAR_STOP_ANGLE, FAKE_LIDAR_MAX_DIST, FAKE_LIDAR_STDDEV, 90);

    constexpr slam::Pose SCANNER_OFFSET = {0, 30, 0};
    constexpr unsigned N_PARTICLES = 25;
    slam::MCL mcl(N_PARTICLES, {1600, 900});

    slam::Pose real_position{400, 400, M_PI};

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

    for (auto _ : state)
    {
        const slam::Odometry odom = {2.5, 0.02, 0.02};
        mcl.predict(odom, {0.0005, 0.0005, 0.01, 0.01});
        real_position = slam::sample_motion_model_odometry(odom, real_position);
        mcl.update(scans, FAKE_LIDAR_STDDEV, fake_lidar.max_dist, SCANNER_OFFSET);
    }
}

BENCHMARK(benchmark_mcl)->Iterations(N_BENCHMARK_ITER);

BENCHMARK_MAIN();