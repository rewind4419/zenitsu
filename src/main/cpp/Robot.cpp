#include "Robot.h"
#include "Config.h"

#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() {
    // Constructor - hardware will be initialized in RobotInit()
}

void Robot::RobotInit() {
    // Initialize hardware
    m_drivetrain = std::make_unique<Drivetrain>();
    m_gamepadInput = std::make_unique<GamepadInput>(DRIVER_CONTROLLER_PORT);
    
    // Initialize dashboard
    frc::SmartDashboard::PutBoolean("Field Relative", m_fieldRelative);
    frc::SmartDashboard::PutString("Robot State", "Initialized");
    
    printf("🗾⚡ Zenitsu Robot Initialized! Ready for lightning-fast swerve driving!\n");
}

void Robot::RobotPeriodic() {
    // Update input every loop
    m_gamepadInput->update();
    
    // Update telemetry
    updateDashboard();
}

void Robot::AutonomousInit() {
    frc::SmartDashboard::PutString("Robot State", "Autonomous");
    
    // Stop drivetrain at start of auto
    m_drivetrain->stop();
}

void Robot::AutonomousPeriodic() {
    // Simple autonomous: just stop (no autonomous code for drivetrain-only robot)
    m_drivetrain->stop();
}

void Robot::TeleopInit() {
    frc::SmartDashboard::PutString("Robot State", "Teleop");
    
    printf("⚡ Zenitsu entering teleop mode - Thunder Breathing activated!\n");
}

void Robot::TeleopPeriodic() {
    handleTeleopDrive();
}

void Robot::DisabledInit() {
    frc::SmartDashboard::PutString("Robot State", "Disabled");
    
    // Stop all motors when disabled
    m_drivetrain->stop();
}

void Robot::DisabledPeriodic() {
    // Keep motors stopped while disabled
    m_drivetrain->stop();
}

void Robot::handleTeleopDrive() {
    // Get driver inputs
    Vector2D translation = m_gamepadInput->getDriveTranslation();
    double rotation = m_gamepadInput->getRotation() * NORMAL_TURN_SPEED;
    
    // Create chassis speeds
    ChassisSpeed speeds;
    speeds.vx = translation.x * MAX_DRIVE_SPEED;
    speeds.vy = translation.y * MAX_DRIVE_SPEED;
    speeds.omega = rotation * MAX_ANGULAR_SPEED;
    
    // Drive the robot
    if (m_fieldRelative) {
        // For drivetrain-only robot, we don't have a gyro, so use robot-relative
        // In a full robot, you'd pass the gyro angle here
        m_drivetrain->drive(speeds);
    } else {
        m_drivetrain->drive(speeds);
    }
    
    // Toggle field-relative mode with A button (example - can be customized)
    static bool lastAButton = false;
    bool currentAButton = false;  // m_controller->GetAButton() if you want this feature
    if (currentAButton && !lastAButton) {
        m_fieldRelative = !m_fieldRelative;
        frc::SmartDashboard::PutBoolean("Field Relative", m_fieldRelative);
    }
    lastAButton = currentAButton;
}

void Robot::updateDashboard() {
    // Update drivetrain telemetry
    m_drivetrain->updateTelemetry();
    
    // Update input info
    Vector2D translation = m_gamepadInput->getDriveTranslation();
    frc::SmartDashboard::PutNumber("Drive X", translation.x);
    frc::SmartDashboard::PutNumber("Drive Y", translation.y);
    frc::SmartDashboard::PutNumber("Rotation", m_gamepadInput->getRotation());
    
    // Update control mode info
    frc::SmartDashboard::PutBoolean("Precision Mode", m_gamepadInput->isPrecisionMode());
    frc::SmartDashboard::PutBoolean("Turbo Mode", m_gamepadInput->isTurboMode());
    frc::SmartDashboard::PutNumber("Speed Multiplier", m_gamepadInput->getSpeedMultiplier());
}