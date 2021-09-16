#include "raycast.h"
#include "util.h"

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <cmath>

using namespace slam;

TEST(TestRaycast, TestRaycastExceedMaxDistance)
{
    Eigen::MatrixXf map = Eigen::MatrixXf::Zero(100, 100);
    const Pose expected_output{-1, -1, 0};
    ASSERT_EQ(raycast(map, {50, 50, 0}, 10), expected_output);
}

TEST(TestRaycast, TestRaycastHit)
{
    Eigen::MatrixXf map(3, 3);
    map << 1, 1, 1, 0, 0, 0, 0, 0, 0;
    const Pose result = raycast(map, {1, 1, M_PI / 2}, 3);
    const auto coord = pose_to_image_coordinates(map, {result.x, result.y, 0});
    int i, j;
    std::tie(i, j) = coord;
    ASSERT_EQ(0, i);
    ASSERT_EQ(1, j);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
