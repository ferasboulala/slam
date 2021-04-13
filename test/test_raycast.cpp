#include "raycast.h"

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include <cmath>

using namespace slam;

TEST(TestRaycast, TestRaycastExceedMaxDistance)
{
    Eigen::MatrixXf map = Eigen::MatrixXf::Zero(100, 100);
    const Pose expected_output{ -1, -1, 0 };
    ASSERT_EQ(raycast(map, { 50, 50, 0 }, 10), expected_output);
}

TEST(TestRaycast, TestRaycastHit)
{
    Eigen::MatrixXf map(3, 3);
    map << 1, 1, 1, 0, 0, 0, 0, 0, 0;
    const Pose expected_output{ 1, 0, M_PI / 2 };
    const Pose result = raycast(map, { 1, 1, M_PI / 2 }, 3);
    ASSERT_EQ(std::floor(result.x), std::floor(expected_output.x));
    ASSERT_EQ(std::floor(result.y), std::floor(expected_output.y));
}

TEST(TestRaycast, TestRaycastOutsideBoundaries)
{
    Eigen::MatrixXf map = Eigen::MatrixXf::Zero(100, 100);
    const Pose expected_output{ -1, -1, 0 };
    ASSERT_EQ(raycast(map, { 50, 50, 0 }, 1000), expected_output);
}

int
main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
