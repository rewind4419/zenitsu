#pragma once

/**
 * Hardware Configuration for Zenitsu Swerve Robot
 * Clean, modern configuration for 2025 FRC season
 */

// =============================================================================
// SWERVE DRIVETRAIN CONFIGURATION
// =============================================================================

// Motor CAN IDs
constexpr int FRONT_LEFT_DRIVE_ID = 2;
constexpr int FRONT_RIGHT_DRIVE_ID = 3;
constexpr int BACK_LEFT_DRIVE_ID = 8;
constexpr int BACK_RIGHT_DRIVE_ID = 5;

constexpr int FRONT_LEFT_STEER_ID = 6;
constexpr int FRONT_RIGHT_STEER_ID = 7;
constexpr int BACK_LEFT_STEER_ID = 4;
constexpr int BACK_RIGHT_STEER_ID = 9;

// CANcoder IDs (absolute encoders)
constexpr int FRONT_LEFT_ENCODER_ID = 13;
constexpr int FRONT_RIGHT_ENCODER_ID = 12;
constexpr int BACK_LEFT_ENCODER_ID = 10;
constexpr int BACK_RIGHT_ENCODER_ID = 11;

// =============================================================================
// PHYSICAL DIMENSIONS (meters)
// =============================================================================

constexpr double WHEELBASE_LENGTH = 0.5842;  // Distance between front and rear wheels
constexpr double WHEELBASE_WIDTH = 0.5842;   // Distance between left and right wheels
constexpr double WHEEL_RADIUS = 0.0508;      // 4 inch wheels -> 2 inch radius

// =============================================================================
// GEAR RATIOS
// =============================================================================

constexpr double DRIVE_GEAR_RATIO = 6.75;    // L2 gearing
constexpr double STEER_GEAR_RATIO = 150.0/7.0;  // Steering gear ratio

// =============================================================================
// CONTROL SETTINGS
// =============================================================================

constexpr double MAX_DRIVE_SPEED = 4.0;      // m/s
constexpr double MAX_ANGULAR_SPEED = 2 * 3.14159; // rad/s

// Driver control speeds
constexpr double NORMAL_DRIVE_SPEED = 1.0;   // Normal driving multiplier
constexpr double PRECISION_DRIVE_SPEED = 0.3; // Precision/slow mode
constexpr double TURBO_DRIVE_SPEED = 1.0;    // Turbo/fast mode

constexpr double NORMAL_TURN_SPEED = 0.8;    // Normal turning multiplier

// =============================================================================
// CONTROLLER CONFIGURATION  
// =============================================================================

constexpr int DRIVER_CONTROLLER_PORT = 0;
constexpr double JOYSTICK_DEADBAND = 0.1;    // Ignore inputs smaller than this
