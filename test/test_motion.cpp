#include "motion.h"

#include <gtest/gtest.h>

#include <cmath>

using namespace slam;

TEST(TestMotion, TestMotionModelOdometry)
{
    const Pose expected_output{1, 0, 1};
    ASSERT_EQ(sample_motion_model_odometry({0, 1, 1}, {0, 0, 0}),
              expected_output);
}

TEST(TestMotion, TestMotionModelVelocity)
{
    // Cannot test the case where the angular velocity is 0 with alphas = 0
    // because it is not handled by the model.
    Pose expected_output{0, 0, 1};
    ASSERT_EQ(sample_motion_model_velocity({0, 1}, {0, 0, 0}, 1),
              expected_output);
    expected_output = Pose{0, 0, 0.5};
    ASSERT_EQ(sample_motion_model_velocity({0, 1}, {0, 0, 0}, 0.5),
              expected_output);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
