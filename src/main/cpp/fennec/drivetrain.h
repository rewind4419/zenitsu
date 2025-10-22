#pragma once

#include "maths.h"

#include <rev/SparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>

struct SwerveDriveModule
{
    //  NOTE: This has to be a CANCoder pointer because they're using OOP
    //  and we cant ensure this is a value type
    ctre::phoenix6::hardware::CANcoder* direction_encoder; // absolute encoder
    rev::SparkMax* drive_motor;
    rev::SparkMax* steer_motor;
    rev::SparkRelativeEncoder* steer_encoder; // relative encoder  
    rev::SparkRelativeEncoder* drive_encoder;

    float initial_rotation_offset;
    float previous_drive_encoder;

    v2 initial_vector;
    v2 current_vector;

    v2 tangent_vector;
    v2 center_offset;
};

enum DrivetrainSwerveModule
{
    DrivetrainSwerve_FL = 0,
    DrivetrainSwerve_FR,
    DrivetrainSwerve_BL,
    DrivetrainSwerve_BR,

    DrivetrainSwerve_Count
};

struct Drivetrain
{
    SwerveDriveModule swerve_drives[DrivetrainSwerve_Count];
    bool drivetrain_override = false;
};

struct OdometryFrame
{
    v2      delta_position;
    float   delta_rotation;
};


void initDrivetrain(Drivetrain* drivetrain);
void drivetrainUpdate(Drivetrain* drivetrain, v2 translation, float rotation, float delta_time);
void drivetrainUpdateRawVectors(Drivetrain* drivetrain, v2* target_vectors, float delta_time, bool steering_only);
OdometryFrame getDrivetrainOdometry(Drivetrain* drivetrain);
void printCalibrationData(Drivetrain* drivetrain);