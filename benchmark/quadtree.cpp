#include "quadtree.h"

#include <benchmark/benchmark.h>

#include <random>

static constexpr unsigned N_POINTS = 1000000;

void benchmark_add(benchmark::State &) {}

void benchmark_find(benchmark::State &state) {
    slam::QuadTree tree;
    std::default_random_engine device(0);
	const int MAX_VAL = static_cast<int>(std::sqrt(N_POINTS)) * 10;
    std::uniform_int_distribution distribution(0, MAX_VAL);
    for (unsigned i = 0; i < N_POINTS; ++i) {
        const int x = distribution(device);
        const int y = distribution(device);
        tree.add({x, MAX_VAL - y}, nullptr);
    }

    constexpr unsigned N_BBOXES = 100;
    std::vector<std::tuple<slam::Coordinate, slam::Coordinate>> bboxes;
    bboxes.reserve(N_BBOXES);
    for (unsigned i = 0; i < N_BBOXES; ++i) {
        const unsigned x1 = distribution(device);
        const unsigned y1 = distribution(device);
        const unsigned x2 = distribution(device);
        const unsigned y2 = distribution(device);
        const unsigned top_x = std::min(x1, x2);
        const unsigned top_y = std::max(y1, y2);
        const unsigned bottom_x = std::max(x1, x2);
        const unsigned bottom_y = std::min(y1, y2);
        bboxes.emplace_back(std::tuple<slam::Coordinate, slam::Coordinate>{{top_x, MAX_VAL - top_y}, {bottom_x, MAX_VAL - bottom_y}});
    }

    for (auto _ : state) {
        for (const auto& [upper_left, bottom_right]: bboxes) {
            benchmark::DoNotOptimize(tree.range_query(upper_left, bottom_right));
        }
    }
}

BENCHMARK(benchmark_find)->Iterations(10);
BENCHMARK_MAIN();
