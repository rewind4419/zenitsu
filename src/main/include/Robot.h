#pragma once

#include <memory>
#include <optional>
#include <vector>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/Field2d.h>

// NavX/Studica IMU
#include "studica/AHRS.h"

// Photon vision (simplified for vision-based localization)
#include <photon/PhotonCamera.h>
#include <frc/geometry/Pose3d.h>

// FENNEC includes
#include "fennec/drivetrain.h"
#include "fennec/drivetrain_controller.h"
#include "fennec/gamepad.h"
#include "fennec/localiser.h"
#include "fennec/taskmgr.h"
#include "fennec/config.h"

enum RobotMode {
    ROBOT_AUTO,
    ROBOT_TELEOP,
    ROBOT_DISABLE,
};

// Simplified PhotonParameters struct (only what localiser needs)
struct PhotonParameters {
    std::vector<TagPosition> global_tags;
    std::vector<TagPosition> global_tags_prev;
    int n_tags = 0;
    float calculated_yaw_angle = 0.0f;
    bool first_aim = true;
    bool auto_aim_activated = false;
    float pitch_aim_timer = 0.0f;
    std::vector<frc::Translation3d> tag_rel_robot;
};

// Simple structures needed by taskmgr but not used in drivetrain-only robot
struct SimplifiedShooter {
    bool beam_break_val = false; 
    float control_motor_speed = 0;
    float target_angle = 0;
    float sum_angle = 0;
    bool beam_break_enabled = true;
    float auto_await_timeout = 0.0f;
    float auto_beam_break_timer = 0.0f;
    bool firing_motor_task = false;
    bool firing_mode = false;
    bool brake = false;
    bool ready_fire_amp = false;
    
    // Mock beam break sensor for compatibility
    struct MockBeamBreak {
        bool Get() const { return true; } // Always return true (no note)
    } beam_break;
    
    // Mock firing encoder for compatibility  
    struct MockEncoder {
        double GetVelocity() const { return 0.0; }
    };
    std::unique_ptr<MockEncoder> firing_encoder = std::make_unique<MockEncoder>();
};

struct SimplifiedIntake {
    float intake_speed = 0;
    bool beam_break_val = false;
};

// Essential RobotData structure - drivetrain focused
struct RobotData {
    // Core timing
    float delta_time = 0.02f; // 20ms default
    float enable_time = 0.0f;
    float imu_basis = 0.0f;
    
    // Alliance/field orientation 
    std::optional<frc::DriverStation::Alliance> driverstation_side;
    float side = 0.0f; // 0 = blue, 1 = red
    
    // Core drivetrain components
    Input input;
    Localiser_FirstOrderLag localiser;
    OdometryFrame latest_odometry_frame;
    DrivetrainController drivetrain_controller;
    Drivetrain drivetrain;
    
    // Hardware
    Studica::AHRS* sensor_imu = nullptr;
    
    // Vision system (minimal for localization)
    PhotonParameters photon;
    
    // Driver control
    float driver_speed = 0.0f;
    float held_rotation = 0.0f;
    v2 global_input_translation = {0, 0};
    
    // Simplified subsystems for taskmgr compatibility
    SimplifiedShooter shooter;
    SimplifiedIntake intake;
    
    // Task management
    TaskMgr taskmgr;
    
    // Dashboard
    frc::Field2d field;
    
    // Flags
    bool middle_wheels = false;
};

// Function declarations
void initRobot(RobotData *r, RobotMode mode);
void updateRobot(RobotData *r, float time_step, RobotMode mode);
void robotModeInit(RobotData *robot, RobotMode new_mode);
