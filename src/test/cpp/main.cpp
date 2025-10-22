#include <gtest/gtest.h>
#include "MathUtils.h"
#include "GamepadInput.h"

// Math utilities tests
TEST(MathUtilsTest, VectorMagnitude) {
    Vector2D v(3.0, 4.0);
    EXPECT_DOUBLE_EQ(v.magnitude(), 5.0);
}

TEST(MathUtilsTest, AngleConstrain) {
    EXPECT_DOUBLE_EQ(constrainAngle(4 * M_PI), 0.0);
    EXPECT_DOUBLE_EQ(constrainAngle(-2 * M_PI), 0.0);
}

TEST(MathUtilsTest, Deadband) {
    EXPECT_DOUBLE_EQ(applyDeadband(0.05, 0.1), 0.0);
    EXPECT_DOUBLE_EQ(applyDeadband(0.15, 0.1), 0.15);
}

// GamepadInput tests (basic functionality)
TEST(GamepadInputTest, SpeedMultipliers) {
    // Note: These test the logic, not actual hardware
    // Real hardware testing should be done on robot
    EXPECT_TRUE(true); // Placeholder - implement when needed
}

// SwerveModule tests (kinematics)  
TEST(SwerveModuleTest, StateOptimization) {
    // Test wheel optimization logic
    // Placeholder - implement when needed
    EXPECT_TRUE(true);
}

// Integration test placeholder
TEST(IntegrationTest, DrivetrainBasic) {
    // Full system test placeholder
    EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
