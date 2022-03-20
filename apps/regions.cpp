#include <limits>
#include <opencv2/opencv.hpp>
#include <random>

#include "colors.h"
#include "quadtree.h"
#include "thirdparty/log.h"
#include "util.h"

//#define DRAW    // Comment to get rid of boundaries

static slam::QuadTree tree;
static cv::Mat map;
static cv::Mat map_image_frame;
static slam::Coordinate start{-1, -1};
static slam::Coordinate stop{-1, -1};

void mouse_callback(int event, int x, int y, int, void*)
{
    bool changed = false;
    switch (event)
    {
        case cv::EVENT_LBUTTONDOWN:
            changed = true;
            start = slam::Coordinate{y, x};
            break;
        case cv::EVENT_RBUTTONDOWN:
            changed = true;
            stop = slam::Coordinate{y, x};
            break;
        default:
            return;
    }

    if (changed)
    {
        map_image_frame = map.clone();
#ifdef DRAW
        tree.draw(map_image_frame);
#endif
        if (start.i != -1) cv::circle(map_image_frame, {start.j, start.i}, 7, BLUE, cv::FILLED);
        if (stop.i != -1) cv::circle(map_image_frame, {stop.j, stop.i}, 7, BLUE, cv::FILLED);
    }

    if (changed && start.i != -1 && stop.i != -1)
    {
        slam::Coordinate real_start = start;
        slam::Coordinate real_stop = stop;
        // Make this a util function
        if (start.i > stop.i && start.j > stop.j)
        {
            real_start = stop;
            real_stop = start;
        }
        else if (start.i < stop.i && start.j > stop.j)
        {
            real_start.j = stop.j;
            real_stop.j = start.j;
        }
        else if (start.i > stop.i && start.j < stop.j)
        {
            real_start.i = stop.i;
            real_stop.i = start.i;
        }

        log_info("Querying for (%d, %d), (%d, %d)",
                 real_start.i,
                 real_start.j,
                 real_stop.i,
                 real_stop.j);
        cv::rectangle(
            map_image_frame, {real_start.j, real_start.i}, {real_stop.j, real_stop.i}, GREY, 2);
        for (const auto& match : tree.range_query(real_start, real_stop))
        {
            slam::Coordinate coord;
            void* ptr;
            std::tie(coord, ptr) = match;
            cv::circle(map_image_frame, {coord.j, coord.i}, 6, YELLOW, cv::FILLED);
        }
    }

    cv::imshow("quadtree", map_image_frame);
}

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        log_error("Usage : %s image number_of_points", argv[0]);
        return -1;
    }

    map = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::threshold(map, map, 128, 1.0, cv::THRESH_BINARY);
    map.convertTo(map, CV_32F);
    cv::cvtColor(map, map, cv::COLOR_GRAY2RGB);
    const unsigned N_POINTS = std::stoi(argv[2]);

    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<std::mt19937::result_type> distribution_i(0, map.rows);
    std::uniform_int_distribution<std::mt19937::result_type> distribution_j(0, map.cols);

    for (unsigned p = 0; p < N_POINTS; ++p)
    {
        const int i = distribution_i(generator);
        const int j = distribution_j(generator);
        tree.add({i, j});
        cv::circle(map, {j, i}, 4, RED, cv::FILLED);
    }

    cv::namedWindow("quadtree");
    cv::setMouseCallback("quadtree", mouse_callback);

    map_image_frame = map.clone();
#ifdef DRAW
    tree.draw(map_image_frame);
#endif
    cv::imshow("quadtree", map_image_frame);
    while (cv::waitKey(33) != 113)
    {
    }

    return 0;
}
