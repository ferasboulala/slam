#include "quadtree.h"

#include <benchmark/benchmark.h>

#include <random>

static constexpr unsigned N_POINTS = 1000000;
static constexpr int MAX_VAL = 10000;

void benchmark_insert(benchmark::State &state) {
    std::default_random_engine device(0);
    std::uniform_int_distribution distribution(0, MAX_VAL);
    std::vector<std::tuple<unsigned, unsigned>> points;
    points.reserve(N_POINTS);
    for (unsigned i = 0; i < N_POINTS; ++i) {
        const unsigned x = distribution(device);
        const unsigned y = distribution(device);
        points.emplace_back(x, y);
    }

    for (auto _ : state) {
    	slam::QuadTree tree;
        for (const auto &[x, y] : points) {
            tree.add({x, MAX_VAL - y}, nullptr);
        }
        benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(N_POINTS * state.iterations());
}

void benchmark_find(benchmark::State &state) {
    slam::QuadTree tree;
    std::default_random_engine device(0);

    std::uniform_int_distribution distribution(0, MAX_VAL);
    for (unsigned i = 0; i < N_POINTS; ++i) {
        const int x = distribution(device);
        const int y = distribution(device);
        tree.add({x, MAX_VAL - y}, nullptr);
    }

    std::uniform_int_distribution distribution2(1, static_cast<int>(0.1 * MAX_VAL));
    static constexpr unsigned N_BBOXES = 1000;
    std::vector<std::tuple<slam::Coordinate, slam::Coordinate>> bboxes;
    bboxes.reserve(N_BBOXES);
    for (unsigned i = 0; i < N_BBOXES; ++i) {
        const unsigned top_x = distribution(device);
        const unsigned bottom_y = distribution(device);
        const unsigned bottom_x = top_x + distribution2(device);
        const unsigned top_y = bottom_y + distribution2(device);
        bboxes.emplace_back(std::tuple<slam::Coordinate, slam::Coordinate>{{top_x, MAX_VAL - top_y}, {bottom_x, MAX_VAL - bottom_y}});
    }

    for (auto _ : state) {
        for (const auto& [upper_left, bottom_right]: bboxes) {
            benchmark::DoNotOptimize(tree.range_query(upper_left, bottom_right).size());
        }
		benchmark::ClobberMemory();
    }
    state.SetItemsProcessed(bboxes.size() * state.iterations());
}

BENCHMARK(benchmark_insert)->Iterations(10);
BENCHMARK(benchmark_find)->Iterations(10);
BENCHMARK_MAIN();
