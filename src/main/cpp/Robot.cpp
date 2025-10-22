#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/DataLogManager.h>

// Network Tables entries for debugging/tuning
nt::GenericEntry* localizerX;
nt::GenericEntry* localizerY; 
nt::GenericEntry* localizerR;
nt::GenericEntry* currentAutoTask;

void initRobot(RobotData *r, RobotMode mode) {
    // Initialize all robot data to safe defaults
    *r = RobotData{};
    
    // Initialize drivetrain
    initDrivetrain(&r->drivetrain);
    initDrivetrainController(&r->drivetrain_controller);
    
    // Initialize task manager
    r->taskmgr = TaskMgr();
    
    // Initialize IMU
    r->sensor_imu = new Studica::AHRS(frc::SPI::Port::kMXP);
    r->sensor_imu->ZeroYaw();
    r->imu_basis = 0;
    
    // Initialize localizer
    r->localiser = {};
    
    // Initialize input
    r->input = {};
    
    // Set up shuffleboard entries for debugging
    localizerX = frc::Shuffleboard::GetTab("Drivetrain").Add("Localizer X", 0.0).GetEntry();
    localizerY = frc::Shuffleboard::GetTab("Drivetrain").Add("Localizer Y", 0.0).GetEntry();
    localizerR = frc::Shuffleboard::GetTab("Drivetrain").Add("Localizer Rotation", 0.0).GetEntry();
    currentAutoTask = frc::Shuffleboard::GetTab("State").Add("Current Auto Task ID", 0).GetEntry();
    
    // Set up field display
    frc::SmartDashboard::PutData("Field", &r->field);
}

void robotModeInit(RobotData *robot, RobotMode new_mode) {
    // Reset timing
    robot->enable_time = 0;
    
    // Reset localizer
    robot->localiser = {};
    
    // Reset IMU
    robot->sensor_imu->ZeroYaw();
    robot->imu_basis = 0;
    
    // Reset drivetrain controller to safe state
    robot->drivetrain_controller.mode = DRIVECTRL_VELOCITY;
    robot->drivetrain_controller.ctrl.velocity.velocity = {0, 0};
    robot->drivetrain_controller.ctrl.velocity.angular_velocity = 0;
    
    // Get alliance color for field orientation
    robot->driverstation_side = frc::DriverStation::GetAlliance();
    if (robot->driverstation_side == frc::DriverStation::kRed) 
        robot->side = 1.0f;
    else if (robot->driverstation_side == frc::DriverStation::kBlue) 
        robot->side = 0.0f;
    else 
        robot->side = 0.0f; // Default to blue if unknown
        
    // Initialize localizer based on alliance
    initLocaliser(robot);
}

void updateRobot(RobotData *r, float time_step, RobotMode mode) {
    // Update timing
    r->delta_time = time_step;
    
    if (mode == ROBOT_DISABLE) {
        // In disabled mode, just update timing but don't move
        return;
    }
    
    r->enable_time += r->delta_time;
    
    // Update alliance info 
    r->driverstation_side = frc::DriverStation::GetAlliance();
    if (r->driverstation_side == frc::DriverStation::kRed) 
        r->side = 1.0f;
    else if (r->driverstation_side == frc::DriverStation::kBlue) 
        r->side = 0.0f;
    
    // Update gamepad input
    updateGamepad(&r->input);
    
    // Get drivetrain odometry
    r->latest_odometry_frame = getDrivetrainOdometry(&r->drivetrain);
    
    // Update localizer
    stepLocaliser(r);
    
    // Reset drivetrain controller targets (they'll be set by teleop/auto code)
    if (r->drivetrain_controller.mode == DRIVECTRL_VELOCITY) {
        r->drivetrain_controller.ctrl.velocity.velocity = v2{0, 0};
        r->drivetrain_controller.ctrl.velocity.angular_velocity = 0;
    } else if (r->drivetrain_controller.mode == DRIVECTRL_THROTTLE) {
        r->drivetrain_controller.ctrl.throttle.throttle = v2{0, 0};
        r->drivetrain_controller.ctrl.throttle.angular_throttle = 0;
    }
    
    // Handle basic teleop driving (if in teleop mode)
    if (mode == ROBOT_TELEOP) {
        // Set driver speed based on trigger/button state
        float speed_multiplier = CFG_DRIVER_SPEED_NORMAL;
        if (r->input.driver.button_left_bumper.held) {
            speed_multiplier = CFG_DRIVER_SPEED_SURGERY; // Slow/precise mode
        } else if (r->input.driver.button_right_bumper.held) {
            speed_multiplier = CFG_DRIVER_SPEED_SPRINT; // Fast mode
        }
        
        // Get translation from left stick
        v2 translation = {
            r->input.driver.stick_left.x * speed_multiplier,
            r->input.driver.stick_left.y * speed_multiplier
        };
        
        // Get rotation from right stick
        float rotation = r->input.driver.stick_right.x * CFG_DRIVER_SPEED_NORMAL_ROT;
        
        // Apply field-relative driving
        float imu_angle = degToRad(r->sensor_imu->GetYaw()) + r->imu_basis;
        r->global_input_translation = rotate(translation, -imu_angle);
        
        // Set drivetrain controller targets
        r->drivetrain_controller.mode = DRIVECTRL_VELOCITY;
        r->drivetrain_controller.ctrl.velocity.velocity = r->global_input_translation;
        r->drivetrain_controller.ctrl.velocity.angular_velocity = rotation;
    }
    
    // Update task manager (handles autonomous tasks)
    updateManager(&r->taskmgr, r);
    
    // Update drivetrain controller
    updateDrivetrainController(r, &r->drivetrain_controller, &r->drivetrain, r->latest_odometry_frame, r->delta_time);
    
    // Update dashboard values
    localizerX->SetDouble(r->localiser.pose_estimate.position.x);
    localizerY->SetDouble(r->localiser.pose_estimate.position.y);
    localizerR->SetDouble(r->localiser.pose_estimate.rotation);
    
    // Update field display
    r->field.SetRobotPose(frc::Pose2d(
        units::meter_t(r->localiser.pose_estimate.position.x),
        units::meter_t(r->localiser.pose_estimate.position.y), 
        frc::Rotation2d(units::radian_t(r->localiser.pose_estimate.rotation))
    ));
}
