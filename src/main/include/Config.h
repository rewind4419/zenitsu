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
constexpr int FRONT_LEFT_ENCODER_ID = 12;
constexpr int FRONT_RIGHT_ENCODER_ID = 13;
constexpr int BACK_LEFT_ENCODER_ID = 10;
constexpr int BACK_RIGHT_ENCODER_ID = 11;

// =============================================================================
// PHYSICAL DIMENSIONS (meters)
// =============================================================================

constexpr double WHEELBASE_LENGTH = 0.45;  // Distance between front and rear wheels
constexpr double WHEELBASE_WIDTH = 0.45;   // Distance between left and right wheels
constexpr double WHEEL_RADIUS = 0.0508;      // 4 inch wheels -> 2 inch radius

// =============================================================================
// GEAR RATIOS
// =============================================================================

constexpr double DRIVE_GEAR_RATIO = 3.0;    // L2 gearing (was formerly 6.75 in fennec, measured 3.0 rotations)
constexpr double STEER_GEAR_RATIO = 12.8;    // Steering gear ratio

// =============================================================================
// CONTROL SETTINGS
// =============================================================================

constexpr double MAX_DRIVE_SPEED = 2.0;      // m/s - reasonable max speed
constexpr double MAX_ANGULAR_SPEED = 1 * 3.14159; // rad/s - reasonable max rotation

// Driver control speeds
constexpr double NORMAL_DRIVE_SPEED = 0.5;   // Normal driving multiplier (1.0 m/s actual)
constexpr double TURBO_DRIVE_SPEED = 1.0;    // Turbo/fast mode (R1) (2.0 m/s actual)

constexpr double NORMAL_TURN_SPEED = 0.5;    // Normal turning multiplier
constexpr double TURBO_TURN_SPEED = 0.5;     // Turbo turning multiplier (R1) - same as normal

// =============================================================================
// SAFETY CONFIGURATION
// =============================================================================

constexpr double CONTROLLER_TIMEOUT = 0.5;  // Controller disconnect timeout (seconds)

// =============================================================================
// CONTROL GAINS (steer P controller)
// =============================================================================

constexpr double STEER_P_GAIN = 0.3;        // Simple P control for steering (unit: output/radian) - lowered to prevent jitter

// Prefer motor-controller onboard PID for steering if available
#define USE_ONBOARD_STEER_PID 0  // Disabled: use software P control

// =============================================================================
// INPUT SLEW RATES (tune for driver feel)
// =============================================================================

constexpr double SLEW_RATE_VX = 3.0;     // m/s per second
constexpr double SLEW_RATE_VY = 3.0;     // m/s per second
constexpr double SLEW_RATE_OMEGA = 6.0;  // rad/s per second

// =============================================================================
// ENCODER OFFSETS (radians) - set after calibration
// =============================================================================

constexpr double FRONT_LEFT_ENCODER_OFFSET  = -2.0893;
constexpr double FRONT_RIGHT_ENCODER_OFFSET = -0.9649;
constexpr double BACK_LEFT_ENCODER_OFFSET   = 0.1289;
constexpr double BACK_RIGHT_ENCODER_OFFSET  = 2.253;

// =============================================================================
// PLAYSTATION CONTROLLER MAPPING
// =============================================================================

constexpr int PS_AXIS_LEFT_X   = 0;
constexpr int PS_AXIS_LEFT_Y   = 1;
constexpr int PS_AXIS_RIGHT_X  = 2;

constexpr int PS_BTN_L1        = 5;
constexpr int PS_BTN_R1        = 6;
constexpr int PS_BTN_SHARE     = 9;
constexpr int PS_BTN_OPTIONS   = 10;
constexpr int PS_BTN_PS        = 14;

// =============================================================================
// SENSOR CONFIGURATION
// =============================================================================

constexpr int NAVX_PORT = 0;  // MXP port on roboRIO

// =============================================================================
// CONTROLLER CONFIGURATION (PlayStation DualShock)
// =============================================================================

constexpr int DRIVER_CONTROLLER_PORT = 0;
constexpr double JOYSTICK_DEADBAND = 0.1;    // Ignore inputs smaller than this

// PlayStation DualShock Controller Layout:
// - Left stick: Drive translation (forward/back, left/right)
// - Right stick: Rotation (left/right only)
// - R1 (button 6): Turbo mode (hold for 2.0 m/s drive; default is 1.0 m/s drive)
// - PS Button (button 14): Toggle field-relative mode
// 
// Diagnostic modes:
// - Options + L1: Drive motors only at 30%
// - Options + R1: Steer motors only at 20%
// - Share + Options: 4s steer test, then 4s drive test
