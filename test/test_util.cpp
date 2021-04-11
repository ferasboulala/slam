#include "util.h"

#include <gtest/gtest.h>

using namespace slam;

TEST(TestUtil, TestPDFNormalDistribution) { 
    ASSERT_DOUBLE_EQ(0.3989422804014327, pdf_normal_distribution(1, 0));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
