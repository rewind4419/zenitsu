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
    
    // Initialize NavX via Studica library (if available in this environment)
    #if NAVX_AVAILABLE
    try {
        m_navx = std::make_unique<studica::AHRS>(studica::AHRS::NavXComType::kMXP_SPI);
        printf("⚡ NavX gyroscope initialized via Studica library\n");
    } catch (std::exception& ex) {
        printf("⚡ NavX initialization failed: %s\n", ex.what());
    }
    #else
    printf("⚡ NavX headers not available in this build environment (CI sim). Skipping NavX init.\n");
    #endif
    
    // Initialize dashboard
    frc::SmartDashboard::PutBoolean("Field Relative", m_fieldRelative);
    frc::SmartDashboard::PutBoolean("Emergency Stop", m_emergencyStop);
    frc::SmartDashboard::PutBoolean("Calibration Mode", m_calibrationMode);
    frc::SmartDashboard::PutString("Robot State", "Initialized");

    // Show field view for odometry (CONFIGURABLE: use during autos/tuning)
    frc::SmartDashboard::PutData("Field", &m_field);
    
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

    // Update odometry and field visualization when NavX is present
    #if NAVX_AVAILABLE
    auto pose = m_drivetrain->updateOdometry(m_navx->IsConnected() ? degreesToRadians(m_navx->GetYaw()) : 0.0);
    m_field.SetRobotPose(pose);
    #endif
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
    
    // Compute target chassis speeds (CONFIGURABLE: tune rates in Config.h)
    const double target_vx = translation.x * MAX_DRIVE_SPEED;
    const double target_vy = translation.y * MAX_DRIVE_SPEED;
    const double target_omega = rotation * MAX_ANGULAR_SPEED * NORMAL_TURN_SPEED;

    // Apply simple manual slew limiting using delta time
    double now = frc::Timer::GetFPGATimestamp().value();
    double dt = now - m_lastUpdateSec;
    m_lastUpdateSec = now;
    auto clampRate = [dt](double prev, double target, double rateLimit) {
        double maxDelta = rateLimit * std::max(0.0, dt);
        double delta = target - prev;
        if (delta > maxDelta) delta = maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;
        return prev + delta;
    };

    m_prevVx    = clampRate(m_prevVx,    target_vx,    SLEW_RATE_VX);
    m_prevVy    = clampRate(m_prevVy,    target_vy,    SLEW_RATE_VY);
    m_prevOmega = clampRate(m_prevOmega, target_omega, SLEW_RATE_OMEGA);

    ChassisSpeed speeds;
    speeds.vx    = m_prevVx;
    speeds.vy    = m_prevVy;
    speeds.omega = m_prevOmega;
    
    // Drive the robot
    #if NAVX_AVAILABLE
    if (m_fieldRelative && m_navx->IsConnected()) {
        // Use NavX for true field-relative driving
        double gyroAngle = degreesToRadians(m_navx->GetYaw());
        m_drivetrain->driveFieldRelative(speeds, gyroAngle);
    } else {
        // Robot-relative driving
        m_drivetrain->drive(speeds);
    }
    #else
    // Robot-relative driving (NavX not available in this build)
    m_drivetrain->drive(speeds);
    #endif
    
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
        #if NAVX_AVAILABLE
        m_navx->Reset();
        printf("⚡ NavX gyroscope reset to zero\n");
        #else
        printf("⚡ Gyro reset requested (NavX not available in this build)\n");
        #endif
    }
    lastOptionsButton = currentOptionsButton;
}

void Robot::updateDashboard() {
    // Update drivetrain telemetry
    m_drivetrain->updateTelemetry();
    
    // Update NavX telemetry
    #if NAVX_AVAILABLE
    if (m_navx->IsConnected()) {
        frc::SmartDashboard::PutNumber("Gyro Yaw", m_navx->GetYaw());
        frc::SmartDashboard::PutNumber("Gyro Pitch", m_navx->GetPitch());
        frc::SmartDashboard::PutNumber("Gyro Roll", m_navx->GetRoll());
        frc::SmartDashboard::PutBoolean("NavX Connected", true);
    } else {
        frc::SmartDashboard::PutBoolean("NavX Connected", false);
    }
    #else
    frc::SmartDashboard::PutString("Gyro Status", "NavX headers not present in CI build");
    #endif
    
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
    
    // Display current raw encoder positions (radians) for copy/paste
    auto raw = m_drivetrain->getRawModuleAngles();
    printf("⚡ CALIBRATION - Use these as offsets in Config.h (radians):\n");
    printf("   FRONT_LEFT_ENCODER_OFFSET  = %.6f\n", raw[0]);
    printf("   FRONT_RIGHT_ENCODER_OFFSET = %.6f\n", raw[1]);
    printf("   BACK_LEFT_ENCODER_OFFSET   = %.6f\n", raw[2]);
    printf("   BACK_RIGHT_ENCODER_OFFSET  = %.6f\n", raw[3]);
}