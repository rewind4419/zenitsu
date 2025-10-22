#pragma once
#include "maths.h"

// This is the robot's configuration file
// It will have constants and ID's that we can change all in one place

constexpr float CFG_DELTA_TIME = 0.02f;

//////////////////////////////// Drivetrain ////////////////////////////////


//////////////////
//3    FRONT   1//
//              //
//             R//
//L            I//
//E            G//
//F            H//
//T            T//
//              //
//4    BACK    2//
//////////////////`


//            MOTOR NAME                                MOTOR LABEL
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_BL = 2; // D4
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_FL = 3; // D3 CHANGE TO ID 3
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_FR = 8; // D1
constexpr int CFG_CAN_DRIVETRAIN_DRIVE_MOTOR_BR = 5; // D2

constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_BL = 6; // S4
constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_FL = 7; // S3
constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_FR = 4; // S1
constexpr int CFG_CAN_DRIVETRAIN_STEER_MOTOR_BR = 9; // S2

//            ENCODER NAME                                 Encoder LABEL
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_BL = 13; // S4
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_FL = 12; // S3
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_FR = 10; // S1
constexpr int CFG_CAN_DRIVETRAIN_STEER_ENCODER_BR = 11; // S2


//              DRIVETRAIN MOTOR OFFSET                     OFFSET (in radians)

constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FL = 3.707632 + M_PI;
constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FR = 2.291767 + M_PI;
constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BL = 6.238700 + M_PI;
constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BR = 2.540272 + M_PI;


// // becky drivetrain values before we swapped the FR swerve module, disabled 10/25/2024
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FL = 3.701496 + M_PI;
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_FR = 3.232098 + M_PI;
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BL = 6.203418 + M_PI;
// constexpr float CFG_DRIVETRAIN_INITIAL_ROTATION_OFFSET_BR = 2.528000 + M_PI;


// fl -- 2.495787
// fr -- 2.133767
// bl -- 3.253573
// br -- 1.512505




constexpr float CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_HORIZONTAL = 26 * INCH_TO_METER;
constexpr float CFG_DRIVETRAIN_DISTANCE_BETWEEN_WHEELS_VERTICAL   = 26 * INCH_TO_METER;

// NOTE: drivetrain gear ratios are measured in Motor Rotations : Final Wheel Rotations

constexpr float CFG_DRIVETRAIN_DRIVE_RATIO      = 6.75; // gear ratio of the drive wheel
constexpr float CFG_DRIVETRAIN_STEER_RATIO      = 12.8; // gear ratio of the steer motor
constexpr float CFG_DRIVETRAIN_WHEEL_RADIUS     = 4; // measured in inches


constexpr float CFG_DRIVETRAIN_TARGET_VECTOR_EPSILON = 0.05; // if the move vector is below a certain amount, dont bother moving the wheel to avoid super small numbers causing weird rotations


constexpr float CFG_DRIVETRAIN_ANTIDRIFT = 0.0; // @Try, lets see if 0.1 works?

//////////////////////////////// Driver ////////////////////////////////
// constexpr float CFG_DRIVER_SPEED_NORMAL  = 6 * 0.7;
// constexpr float CFG_DRIVER_SPEED_SURGERY = 6 * 0.59;
// constexpr float CFG_DRIVER_SPEED_SPRINT  = 6 * 1;

// constexpr float CFG_DRIVER_SPEED_SURGERY_ROT = 10;
// constexpr float CFG_DRIVER_SPEED_NORMAL_ROT = 20;
// constexpr float CFG_DRIVER_SPEED_SPRINT_ROT = 20;


// old
constexpr float CFG_DRIVER_SPEED_NORMAL  = 0.7;
constexpr float CFG_DRIVER_SPEED_NORMAL_ROT = 0.7;

constexpr float CFG_DRIVER_SPEED_SURGERY = 0.4;
constexpr float CFG_DRIVER_SPEED_SURGERY_ROT = 0.4;

constexpr float CFG_DRIVER_SPEED_SPRINT  = 1;
constexpr float CFG_DRIVER_SPEED_SPRINT_ROT = 0.7;




constexpr float CFG_DRIVER_ABSOLUTE_ROTATION_REACTIVENESS = 0.8; // 0.5 ....  higher reactivity = closer to 0, straight reactiveness curve = 1

constexpr float CFG_DRIVER_ADJUSTMENT_ROTATION_SENSITIVITY = 0.5;


//////////////////////////////// Intake ////////////////////////////////

constexpr int CFG_INTAKE_MOTOR = 10;

constexpr int CFG_INTAKE_BB_DIO = 1;

constexpr float CFG_INTAKE_MAX_SPEED = 0.8;



//////////////////////////////// Shooter ////////////////////////////////

constexpr int CFG_SHOOTER_CONTROL_MOTOR = 30;
constexpr int CFG_SHOOTER_FIRING_MOTOR = 31;
constexpr int CFG_SHOOTER_FIRING_MOTOR_2 = 35;

constexpr int CFG_SHOOTER_AXIS_LEFT = 21;
constexpr int CFG_SHOOTER_AXIS_RIGHT = 22;
constexpr int CFG_SHOOTER_AXIS_MOTOR_COUNT = 2;

constexpr int CFG_SHOOTER_BB_DIO = 1;

constexpr float CFG_SHOOTER_ANGLE_OFFSET = 0.296706; //In radians - 17 degrees
constexpr float CFG_SHOOTER_AXIS_THROTTLE = 0.45f;
constexpr float CFG_SHOOTER_CONTROL_MAX_SPEED = 0.3f;
constexpr float CFG_SHOOTER_MAX_ANGLE = 11.774826; 
constexpr float CFG_SHOOTER_ANGLE_RANGE = 1.6057;
constexpr float CFG_SHOOTER_START_ANGLE = 0;
constexpr float CFG_SHOOTER_MAX_FIRING_SPEED = 1.0f;
constexpr float CFG_SHOOTER_PERPENDICULAR_THROTTLE = 0.07f;
constexpr float CFG_SHOOTER_RADIUS = 18.5 * INCH_TO_METER; // Change
constexpr float CFG_SHOOTER_AXIS_HEIGHT = 9 * INCH_TO_METER; //Change
constexpr float CFG_SHOOTER_DIST_CAM_TO_AXIS = 18 * INCH_TO_METER; //Change

constexpr float CFG_SHOOTER_AMP_SCORE_TARGET_VELOCITY = -2000.0f;

constexpr float CFG_TARGET_VELOCITY_FIRING_WHEELS = 5600;

//////////////////////////////// Localiser ////////////////////////////////
constexpr v2 CFG_APRILTAG_CAMERA_OFFSET = { 0, -13 * INCH_TO_METER };

constexpr int CFG_APRILTAG_HIGHEST_ID = 8;


constexpr v2 CFG_CENTER_OF_GRAVITY = { 0, -0.021 };
constexpr float CFG_CG_ANTIDRIFT_STRENGTH = 1;

constexpr float CFG_DRIVETRAIN_ODO_MULTIPLIER = 1.0f;


///// experimental /////

constexpr int CFG_LED_STRIP_PIN = 9;
constexpr int CFG_LED_STRIP_LENGTH = 86;

constexpr float CFG_GRAVITATIONAL_CONSTANT = 9.8; // m/s/s

//////// Photon Vision //////// 
constexpr int CFG_APRIL_TAG_COUNT = 16;
constexpr float CFG_MAX_TAG_ALIGN_THROTTLE = 0.4f;

//////// Field Layout //////// 

// Center field x = 8.270620346069336
//Center field y = 5.547867774963379

// Vision Height
constexpr float CFG_SPEAKER_HEIGHT = 81 * INCH_TO_METER;
//constexpr float CFG_SPEAKER_HEIGHT = 92 * INCH_TO_METER;
//constexpr float CFG_SPEAKER_HEIGHT = 87.5413 * INCH_TO_METER;
//constexpr float CFG_SPEAKER_HEIGHT = 84 * INCH_TO_METER;

constexpr float CFG_SPEAKER_HEIGHT_AUTO = 90 * INCH_TO_METER;
//constexpr float CFG_SPEAKER_HEIGHT_AUTO = 83.5 * INCH_TO_METER;
//constexpr float CFG_SPEAKER_HEIGHT_AUTO = 80 * INCH_TO_METER;

//////// Elevator //////// 
constexpr int CFG_ELEVATOR_LEFT_MOTOR = 40; 
constexpr int CFG_ELEVATOR_RIGHT_MOTOR = 41;  
constexpr int CFG_ELEVATOR_ENCODER = 2; //Change  
// constexpr float CFG_ELEVATOR_MIN_ROTATION = 0.27; //Change 
constexpr float CFG_ELEVATOR_MAX_ROTATION = 2.1746f; //Change 
constexpr float CFG_ELEVATOR_RANGE = 11.25f * INCH_TO_METER; //Change 
constexpr float CFG_ELEVATOR_START_ROTATION = 0; 
constexpr float CFG_ELEVATOR_THROTTLE = 1.0f; //Change

// 11.25 + 17.5

constexpr float CFG_CONTROL_PULLER_MAX_SPEED = 0.45f;
constexpr float CFG_INTAKE_PULLER_MAX_SPEED = 0.6f;






