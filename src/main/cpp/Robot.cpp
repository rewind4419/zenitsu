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
    
    // Initialize NavX via Studica library
    try {
        m_navx = std::make_unique<studica::AHRS>(studica::AHRS::NavXComType::kMXP_SPI);
        printf("⚡ NavX gyroscope initialized via Studica library\n");
    } catch (std::exception& ex) {
        printf("⚡ NavX initialization failed: %s\n", ex.what());
    }
    
    // Initialize dashboard
    frc::SmartDashboard::PutBoolean("Field Relative", m_fieldRelative);
    frc::SmartDashboard::PutBoolean("Emergency Stop", m_emergencyStop);
    frc::SmartDashboard::PutBoolean("Calibration Mode", m_calibrationMode);
    frc::SmartDashboard::PutString("Robot State", "Initialized");
    
    printf("🗾⚡ Zenitsu Robot Initialized! Ready for lightning-fast swerve driving with PlayStation controller!\n");
    printf("⚡ NavX gyroscope connected for field-relative driving\n");
    printf("⚡ Press L1+R1+Options to enter calibration mode\n");
    printf("⚡ Press PS button for emergency stop\n");
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
    
    printf("⚡ Zenitsu entering teleop mode\n");
}

void Robot::TeleopPeriodic() {
    // Check for calibration mode toggle (L1 + R1 + Options)
    static bool lastCalibrationToggle = false;
    bool currentCalibrationToggle = m_gamepadInput->isPrecisionMode() && 
                                   m_gamepadInput->isTurboMode() && 
                                   m_gamepadInput->getOptionsButton();
    
    if (currentCalibrationToggle && !lastCalibrationToggle) {
        m_calibrationMode = !m_calibrationMode;
        frc::SmartDashboard::PutBoolean("Calibration Mode", m_calibrationMode);
        printf("⚡ Calibration mode %s\n", m_calibrationMode ? "ENABLED" : "DISABLED");
        
        if (m_calibrationMode) {
            printf("⚡ CALIBRATION MODE: Align all wheels forward and record encoder values\n");
        }
    }
    lastCalibrationToggle = currentCalibrationToggle;
    
    // Handle based on mode
    if (m_calibrationMode) {
        handleCalibration();
    } else {
        handleTeleopDrive();
    }
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
    // Safety checks
    bool controllerConnected = m_gamepadInput->isControllerConnected();
    
    // Handle emergency stop
    static bool lastPSButton = false;
    bool currentPSButton = m_gamepadInput->getPSButton();
    if (currentPSButton && !lastPSButton) {
        m_emergencyStop = !m_emergencyStop;
        frc::SmartDashboard::PutBoolean("Emergency Stop", m_emergencyStop);
        printf("⚡ Emergency stop %s\n", m_emergencyStop ? "ACTIVATED" : "DEACTIVATED");
    }
    lastPSButton = currentPSButton;
    
    // Stop if safety conditions not met
    if (m_emergencyStop || !controllerConnected) {
        m_drivetrain->stop();
        return;
    }
    
    // Get driver inputs
    Vector2D translation = m_gamepadInput->getDriveTranslation();
    double rotation = m_gamepadInput->getRotation() * NORMAL_TURN_SPEED;
    
    // Create chassis speeds
    ChassisSpeed speeds;
    speeds.vx = translation.x * MAX_DRIVE_SPEED;
    speeds.vy = translation.y * MAX_DRIVE_SPEED;
    speeds.omega = rotation * MAX_ANGULAR_SPEED;
    
    // Drive the robot
    if (m_fieldRelative && m_navx->IsConnected()) {
        // Use NavX for true field-relative driving
        double gyroAngle = degreesToRadians(m_navx->GetYaw());
        m_drivetrain->driveFieldRelative(speeds, gyroAngle);
    } else {
        // Robot-relative driving
        m_drivetrain->drive(speeds);
    }
    
    // Toggle field-relative mode with Share button
    static bool lastShareButton = false;
    bool currentShareButton = m_gamepadInput->getShareButton();
    if (currentShareButton && !lastShareButton) {
        m_fieldRelative = !m_fieldRelative;
        frc::SmartDashboard::PutBoolean("Field Relative", m_fieldRelative);
        printf("⚡ Field-relative mode %s\n", m_fieldRelative ? "ENABLED" : "DISABLED");
    }
    lastShareButton = currentShareButton;
    
    // Reset gyro with Options button
    static bool lastOptionsButton = false;
    bool currentOptionsButton = m_gamepadInput->getOptionsButton();
    if (currentOptionsButton && !lastOptionsButton) {
        m_navx->Reset();
        printf("⚡ NavX gyroscope reset to zero\n");
    }
    lastOptionsButton = currentOptionsButton;
}

void Robot::updateDashboard() {
    // Update drivetrain telemetry
    m_drivetrain->updateTelemetry();
    
    // Update NavX telemetry
    if (m_navx->IsConnected()) {
        frc::SmartDashboard::PutNumber("Gyro Yaw", m_navx->GetYaw());
        frc::SmartDashboard::PutNumber("Gyro Pitch", m_navx->GetPitch());
        frc::SmartDashboard::PutNumber("Gyro Roll", m_navx->GetRoll());
        frc::SmartDashboard::PutBoolean("NavX Connected", true);
    } else {
        frc::SmartDashboard::PutBoolean("NavX Connected", false);
    }
    
    // Update input info
    Vector2D translation = m_gamepadInput->getDriveTranslation();
    frc::SmartDashboard::PutNumber("Drive X", translation.x);
    frc::SmartDashboard::PutNumber("Drive Y", translation.y);
    frc::SmartDashboard::PutNumber("Rotation", m_gamepadInput->getRotation());
    
    // Update safety status
    frc::SmartDashboard::PutBoolean("Controller Connected", m_gamepadInput->isControllerConnected());
    
    // Update control mode info
    frc::SmartDashboard::PutBoolean("Precision Mode (L1)", m_gamepadInput->isPrecisionMode());
    frc::SmartDashboard::PutBoolean("Turbo Mode (R1)", m_gamepadInput->isTurboMode());
    frc::SmartDashboard::PutNumber("Speed Multiplier", m_gamepadInput->getSpeedMultiplier());
}

void Robot::handleCalibration() {
    // Stop all drive motors in calibration mode
    m_drivetrain->stop();
    
    // Get raw encoder values for calibration
    // Note: These would be the values to subtract from raw encoder readings
    // to get zero when wheels are pointing forward
    
    frc::SmartDashboard::PutString("Calibration", "ALIGN WHEELS FORWARD");
    frc::SmartDashboard::PutString("Instructions", "1. Manually align all wheels to point forward");
    frc::SmartDashboard::PutString("Instructions2", "2. Record the encoder values below");
    frc::SmartDashboard::PutString("Instructions3", "3. Update Config.h with these offset values");
    frc::SmartDashboard::PutString("Instructions4", "4. Exit calibration mode when done");
    
    // Display current raw encoder positions
    // These values should be used as encoder offsets in Config.h
    printf("⚡ CALIBRATION - Current encoder positions (use as offsets):\n");
    printf("   Front Left:  %.3f\n", 0.0); // Would read actual encoder here
    printf("   Front Right: %.3f\n", 0.0);
    printf("   Back Left:   %.3f\n", 0.0);
    printf("   Back Right:  %.3f\n", 0.0);
}