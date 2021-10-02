#include "colors.h"
#include "kdtree.h"
#include "thirdparty/log.h"
#include "util.h"

#include <opencv2/opencv.hpp>

#include <limits>
#include <random>

#define KDTREE  // Comment to use linear search
#define DRAW    // Comment to get rid of boundaries

static slam::KDTree tree;
static cv::Mat map;
static cv::Mat map_image_frame;

void mouse_callback(int event, int x, int y, int, void *)
{
    if (event != cv::EVENT_LBUTTONDOWN) return;
    log_info("Querying for %d %d", y, x);
    map_image_frame = map.clone();
#ifdef DRAW
    tree.draw(map_image_frame);
#endif
#ifdef KDTREE
    const slam::Coordinate nn = std::get<0>(tree.nearest_neighbor({y, x}));
#else
    slam::Coordinate nn;
    double dist = std::numeric_limits<double>::max();
    for (const slam::Coordinate &point : tree.list_points())
    {
        if (slam::euclidean_distance(point, {y, x}) < dist)
        {
            dist = slam::euclidean_distance(point, {y, x});
            nn = point;
        }
    }
#endif
    log_info("Nearest point is %d %d", nn.i, nn.j);
    cv::line(map_image_frame, {nn.j, nn.i}, {x, y}, BLUE * 255, 3);
    cv::imshow("kdt", map_image_frame);
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        log_error("Usage : %s number_of_points", argv[0]);
        return -1;
    }

    map = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    cv::cvtColor(map, map, cv::COLOR_GRAY2RGB);
    const unsigned N_POINTS = std::stoi(argv[2]);

    std::default_random_engine generator;
    generator.seed(std::chrono::system_clock::now().time_since_epoch().count());
    std::uniform_int_distribution<std::mt19937::result_type> distribution_i(
        0, map.rows);
    std::uniform_int_distribution<std::mt19937::result_type> distribution_j(
        0, map.cols);

    for (unsigned p = 0; p < N_POINTS; ++p)
    {
        const int i = distribution_i(generator);
        const int j = distribution_j(generator);
        tree.add({i, j}, nullptr);
        cv::circle(map, {j, i}, 4, RED * 255, cv::FILLED);
    }

    cv::namedWindow("kdt");
    cv::setMouseCallback("kdt", mouse_callback);

    map_image_frame = map.clone();
#ifdef DRAW
    tree.draw(map_image_frame);
#endif
    cv::imshow("kdt", map_image_frame);
    while (cv::waitKey(33) != 113)
    {
    }

    return 0;
}
