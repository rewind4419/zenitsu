#include <gtest/gtest.h>

// Basic test to verify the test framework is working
TEST(BasicTest, AlwaysPass) {
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
