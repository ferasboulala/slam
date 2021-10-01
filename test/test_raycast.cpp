#include "raycast.h"
#include "util.h"

#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include <cmath>

using namespace slam;

TEST(TestRaycast, TestRaycastExceedMaxDistance)
{
    cv::Mat map(cv::Size(100, 100), CV_32S, cv::Scalar(0));
    const Pose expected_output{-1, -1, 0};
    ASSERT_EQ(raycast<int>(map, {50, 50, 0}, 10), expected_output);
}

TEST(TestRaycast, TestRaycastHit)
{
    int data[] = {1, 1, 1, 0, 0, 0, 0, 0, 0};
    cv::Mat map(cv::Size(3, 3), CV_32S, reinterpret_cast<void *>(data));
    const Pose result = raycast<int>(map, {1, 1, M_PI / 2}, 3);
    const auto coord = pose_to_image_coordinates(map, {result.x, result.y, 0});
    int i, j;
    std::tie(i, j) = coord;
    ASSERT_EQ(0, i);
    ASSERT_EQ(1, j);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
