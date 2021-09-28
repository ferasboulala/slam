#include "mapping/util.h"

#include <gtest/gtest.h>

#include <cmath>

using namespace slam;

TEST(TestUtil, TestPDFNormalDistribution)
{
    ASSERT_DOUBLE_EQ(0.3989422804014327, pdf_normal_distribution(1, 0));
}

TEST(TestUtil, TestPDFNormalDistributionClamp)
{
    ASSERT_DOUBLE_EQ(0.0, pdf_normal_distribution_clamp(1, 5));
    ASSERT_DOUBLE_EQ(pdf_normal_distribution(1, 0),
                     pdf_normal_distribution_clamp(1, 0));
}

TEST(TestUtil, TestPDFTriangularDistribution)
{
    ASSERT_DOUBLE_EQ(0.40824829046386307, pdf_triangular_distribution(1, 0));
}

TEST(TestUtil, TestNormalizeAngle)
{
    ASSERT_DOUBLE_EQ(M_PI, normalize_angle(M_PI));
    ASSERT_DOUBLE_EQ(0, normalize_angle(2 * M_PI));
    ASSERT_DOUBLE_EQ(0, normalize_angle(-2 * M_PI));
}

TEST(TestUtil, TestSampleNormalDistribution)
{
    ASSERT_DOUBLE_EQ(0, sample_normal_distribution(0));
}

TEST(TestUtil, TestPoseToImageCoordinates)
{
    const cv::Mat map(cv::Size(100, 100), CV_32S, cv::Scalar(0));
    int x, y;

    auto ret = pose_to_image_coordinates(map, {0, 0, 0});
    std::tie(y, x) = ret;
    ASSERT_DOUBLE_EQ(0, x);
    ASSERT_DOUBLE_EQ(99, y);

    ret = pose_to_image_coordinates(map, {20, 60, 0});
    std::tie(y, x) = ret;
    ASSERT_DOUBLE_EQ(20, x);
    ASSERT_DOUBLE_EQ(39, y);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
