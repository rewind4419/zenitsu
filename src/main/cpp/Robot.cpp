#include "Robot.h"
#include "Config.h"

#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() {
    // Constructor - hardware will be initialized in RobotInit()
}

void Robot::RobotInit() {
    // Initialize DataLogManager for AdvantageScope analysis
    frc::DataLogManager::Start();
    printf("DataLogManager started for AdvantageScope logging\n");
    
    // Initialize hardware
    m_drivetrain = std::make_unique<Drivetrain>();
    m_gamepadInput = std::make_unique<GamepadInput>(DRIVER_CONTROLLER_PORT);
    
    // Initialize NavX via Studica library (if available in this environment)
    #if NAVX_AVAILABLE
    try {
        m_navx = std::make_unique<studica::AHRS>(studica::AHRS::NavXComType::kMXP_SPI);
        printf("NavX gyroscope initialized via Studica library\n");
    } catch (std::exception& ex) {
        printf("NavX initialization failed: %s\n", ex.what());
    }
    #else
    printf("NavX headers not available in this build environment (CI sim). Skipping NavX init.\n");
    #endif
    
    // Initialize dashboard
    frc::SmartDashboard::PutBoolean("Field Relative", m_fieldRelative);
    frc::SmartDashboard::PutString("Robot State", "Initialized");

    // Show field view for odometry (CONFIGURABLE: use during autos/tuning)
    frc::SmartDashboard::PutData("Field", &m_field);
    
    printf("Zenitsu Robot Initialized! Ready for lightning-fast swerve driving with PlayStation controller!\n");
    printf("NavX gyroscope connected for field-relative driving\n");
}

void Robot::RobotPeriodic() {
    // Update input every loop
    m_gamepadInput->update();
    
    // Update telemetry
    updateDashboard();
    
    // Log drivetrain performance for AdvantageScope
    logDrivetrainPerformance();

    // Update odometry and field visualization when NavX is present
    #if NAVX_AVAILABLE
    auto pose = m_drivetrain->updateOdometry(m_navx->IsConnected() ? degreesToRadians(m_navx->GetYaw()) : 0.0);
    m_field.SetRobotPose(pose);
    #endif
}

void Robot::AutonomousInit() {
    frc::SmartDashboard::PutString("Robot State", "Autonomous");
    
    // Reset auto timer
    m_autoStartTime = frc::Timer::GetFPGATimestamp().value();
    printf("Starting autonomous: drive backwards at 50%% for 5 seconds\n");
}

void Robot::AutonomousPeriodic() {
    double elapsed = frc::Timer::GetFPGATimestamp().value() - m_autoStartTime;
    
    if (elapsed < 5.0) {
        // Drive backwards at 50% speed for 5 seconds
        ChassisSpeed speeds;
        speeds.vx = -0.5 * MAX_DRIVE_SPEED;  // Backwards (negative)
        speeds.vy = 0.0;                      // No strafe
        speeds.omega = 0.0;                   // No rotation
        m_drivetrain->drive(speeds);
        
        // Print status every 0.5 seconds
        static double lastPrint = 0.0;
        if (elapsed - lastPrint > 0.5) {
            printf("Auto: %.1fs - driving backwards\n", elapsed);
            lastPrint = elapsed;
        }
    } else {
        // Stop after 5 seconds
        m_drivetrain->stop();
    }
}

void Robot::TeleopInit() {
    frc::SmartDashboard::PutString("Robot State", "Teleop");
    
    // Print calibration data on teleop start
    printf("\n========== CALIBRATION DATA ==========\n");
    auto raw = m_drivetrain->getRawModuleAngles();
    printf("Copy these values to Config.h if recalibrating:\n");
    printf("constexpr double FRONT_LEFT_ENCODER_OFFSET  = %.4f;\n", raw[0]);
    printf("constexpr double FRONT_RIGHT_ENCODER_OFFSET = %.4f;\n", raw[1]);
    printf("constexpr double BACK_LEFT_ENCODER_OFFSET   = %.4f;\n", raw[2]);
    printf("constexpr double BACK_RIGHT_ENCODER_OFFSET  = %.4f;\n", raw[3]);
    printf("======================================\n\n");
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
    // Safety checks
    bool controllerConnected = m_gamepadInput->isControllerConnected();
    
    // Stop if safety conditions not met
    if (!controllerConnected) {
        m_drivetrain->stop();
        return;
    }
    
    // Diagnostic modes (Options + L1/R1, Share + Options)
    bool optionsBtn = m_gamepadInput->getOptionsButton();
    bool shareBtn = m_gamepadInput->getShareButton();
    bool l1Btn = m_gamepadInput->isPrecisionMode();
    bool r1Btn = m_gamepadInput->isTurboMode();
    
    // Options + L1: Drive only at 30%
    if (optionsBtn && l1Btn && !shareBtn) {
        m_drivetrain->driveOnlyDuty(0.3);
        frc::SmartDashboard::PutString("Diag Mode", "Drive Only 30%");
        return;
    }
    
    // Options + R1: Steer only at 20%
    if (optionsBtn && r1Btn && !shareBtn) {
        m_drivetrain->steerOnlyDuty(0.2);
        frc::SmartDashboard::PutString("Diag Mode", "Steer Only 20%");
        return;
    }
    
    frc::SmartDashboard::PutString("Diag Mode", "Off");
    
    // Get driver inputs
    Vector2D translation = m_gamepadInput->getDriveTranslation();
    
    // Apply turn speed multiplier based on mode
    double turnSpeedMultiplier = m_gamepadInput->isTurboMode() ? TURBO_TURN_SPEED : NORMAL_TURN_SPEED;
    double rotation = m_gamepadInput->getRotation() * turnSpeedMultiplier;
    
    // Compute target chassis speeds (CONFIGURABLE: tune rates in Config.h)
    const double target_vx = translation.x * MAX_DRIVE_SPEED;
    const double target_vy = translation.y * MAX_DRIVE_SPEED;
    const double target_omega = rotation * MAX_ANGULAR_SPEED;

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
    
    // Toggle field-relative mode with PS button (only when not in diagnostic mode)
    static bool lastPSButton = false;
    bool currentPSButton = m_gamepadInput->getPSButton();
    if (currentPSButton && !lastPSButton && !optionsBtn && !shareBtn) {
        m_fieldRelative = !m_fieldRelative;
        frc::SmartDashboard::PutBoolean("Field Relative", m_fieldRelative);
        printf("Field-relative mode %s\n", m_fieldRelative ? "ENABLED" : "DISABLED");
    }
    lastPSButton = currentPSButton;
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
    frc::SmartDashboard::PutBoolean("Turbo Mode (R1)", m_gamepadInput->isTurboMode());
    frc::SmartDashboard::PutNumber("Speed Multiplier", m_gamepadInput->getSpeedMultiplier());
}

void Robot::logDrivetrainPerformance() {
    // Get DataLog reference
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    
    // Create static log entries (created once, reused every loop)
    // Chassis speeds
    static wpi::log::DoubleLogEntry logChassisVx(log, "Drivetrain/Chassis/vx");
    static wpi::log::DoubleLogEntry logChassisVy(log, "Drivetrain/Chassis/vy");
    static wpi::log::DoubleLogEntry logChassisOmega(log, "Drivetrain/Chassis/omega");
    
    // Front Left module
    static wpi::log::DoubleLogEntry logFLDesiredSpeed(log, "Drivetrain/FL/Desired/Speed");
    static wpi::log::DoubleLogEntry logFLActualSpeed(log, "Drivetrain/FL/Actual/Speed");
    static wpi::log::DoubleLogEntry logFLDesiredAngle(log, "Drivetrain/FL/Desired/Angle");
    static wpi::log::DoubleLogEntry logFLActualAngle(log, "Drivetrain/FL/Actual/Angle");
    static wpi::log::DoubleLogEntry logFLDriveOutput(log, "Drivetrain/FL/Drive/AppliedOutput");
    static wpi::log::DoubleLogEntry logFLSteerOutput(log, "Drivetrain/FL/Steer/AppliedOutput");
    static wpi::log::DoubleLogEntry logFLDriveCurrent(log, "Drivetrain/FL/Drive/Current");
    static wpi::log::DoubleLogEntry logFLSteerCurrent(log, "Drivetrain/FL/Steer/Current");
    
    // Front Right module
    static wpi::log::DoubleLogEntry logFRDesiredSpeed(log, "Drivetrain/FR/Desired/Speed");
    static wpi::log::DoubleLogEntry logFRActualSpeed(log, "Drivetrain/FR/Actual/Speed");
    static wpi::log::DoubleLogEntry logFRDesiredAngle(log, "Drivetrain/FR/Desired/Angle");
    static wpi::log::DoubleLogEntry logFRActualAngle(log, "Drivetrain/FR/Actual/Angle");
    static wpi::log::DoubleLogEntry logFRDriveOutput(log, "Drivetrain/FR/Drive/AppliedOutput");
    static wpi::log::DoubleLogEntry logFRSteerOutput(log, "Drivetrain/FR/Steer/AppliedOutput");
    static wpi::log::DoubleLogEntry logFRDriveCurrent(log, "Drivetrain/FR/Drive/Current");
    static wpi::log::DoubleLogEntry logFRSteerCurrent(log, "Drivetrain/FR/Steer/Current");
    
    // Back Left module
    static wpi::log::DoubleLogEntry logBLDesiredSpeed(log, "Drivetrain/BL/Desired/Speed");
    static wpi::log::DoubleLogEntry logBLActualSpeed(log, "Drivetrain/BL/Actual/Speed");
    static wpi::log::DoubleLogEntry logBLDesiredAngle(log, "Drivetrain/BL/Desired/Angle");
    static wpi::log::DoubleLogEntry logBLActualAngle(log, "Drivetrain/BL/Actual/Angle");
    static wpi::log::DoubleLogEntry logBLDriveOutput(log, "Drivetrain/BL/Drive/AppliedOutput");
    static wpi::log::DoubleLogEntry logBLSteerOutput(log, "Drivetrain/BL/Steer/AppliedOutput");
    static wpi::log::DoubleLogEntry logBLDriveCurrent(log, "Drivetrain/BL/Drive/Current");
    static wpi::log::DoubleLogEntry logBLSteerCurrent(log, "Drivetrain/BL/Steer/Current");
    
    // Back Right module
    static wpi::log::DoubleLogEntry logBRDesiredSpeed(log, "Drivetrain/BR/Desired/Speed");
    static wpi::log::DoubleLogEntry logBRActualSpeed(log, "Drivetrain/BR/Actual/Speed");
    static wpi::log::DoubleLogEntry logBRDesiredAngle(log, "Drivetrain/BR/Desired/Angle");
    static wpi::log::DoubleLogEntry logBRActualAngle(log, "Drivetrain/BR/Actual/Angle");
    static wpi::log::DoubleLogEntry logBRDriveOutput(log, "Drivetrain/BR/Drive/AppliedOutput");
    static wpi::log::DoubleLogEntry logBRSteerOutput(log, "Drivetrain/BR/Steer/AppliedOutput");
    static wpi::log::DoubleLogEntry logBRDriveCurrent(log, "Drivetrain/BR/Drive/Current");
    static wpi::log::DoubleLogEntry logBRSteerCurrent(log, "Drivetrain/BR/Steer/Current");
    
    // Get data from drivetrain
    auto commandedSpeeds = m_drivetrain->getLastCommandedSpeeds();
    auto actualStates = m_drivetrain->getModuleStates();
    const auto& modules = m_drivetrain->getModules();
    
    // Log chassis speeds
    logChassisVx.Append(commandedSpeeds.vx);
    logChassisVy.Append(commandedSpeeds.vy);
    logChassisOmega.Append(commandedSpeeds.omega);
    
    // Log Front Left (index 0)
    logFLDesiredSpeed.Append(modules[0]->getDesiredSpeed());
    logFLActualSpeed.Append(actualStates[0].speed);
    logFLDesiredAngle.Append(radiansToDegrees(modules[0]->getDesiredAngle()));
    logFLActualAngle.Append(radiansToDegrees(actualStates[0].angle));
    logFLDriveOutput.Append(modules[0]->getDriveAppliedOutput());
    logFLSteerOutput.Append(modules[0]->getSteerAppliedOutput());
    logFLDriveCurrent.Append(modules[0]->getDriveOutputCurrent());
    logFLSteerCurrent.Append(modules[0]->getSteerOutputCurrent());
    
    // Log Front Right (index 1)
    logFRDesiredSpeed.Append(modules[1]->getDesiredSpeed());
    logFRActualSpeed.Append(actualStates[1].speed);
    logFRDesiredAngle.Append(radiansToDegrees(modules[1]->getDesiredAngle()));
    logFRActualAngle.Append(radiansToDegrees(actualStates[1].angle));
    logFRDriveOutput.Append(modules[1]->getDriveAppliedOutput());
    logFRSteerOutput.Append(modules[1]->getSteerAppliedOutput());
    logFRDriveCurrent.Append(modules[1]->getDriveOutputCurrent());
    logFRSteerCurrent.Append(modules[1]->getSteerOutputCurrent());
    
    // Log Back Left (index 2)
    logBLDesiredSpeed.Append(modules[2]->getDesiredSpeed());
    logBLActualSpeed.Append(actualStates[2].speed);
    logBLDesiredAngle.Append(radiansToDegrees(modules[2]->getDesiredAngle()));
    logBLActualAngle.Append(radiansToDegrees(actualStates[2].angle));
    logBLDriveOutput.Append(modules[2]->getDriveAppliedOutput());
    logBLSteerOutput.Append(modules[2]->getSteerAppliedOutput());
    logBLDriveCurrent.Append(modules[2]->getDriveOutputCurrent());
    logBLSteerCurrent.Append(modules[2]->getSteerOutputCurrent());
    
    // Log Back Right (index 3)
    logBRDesiredSpeed.Append(modules[3]->getDesiredSpeed());
    logBRActualSpeed.Append(actualStates[3].speed);
    logBRDesiredAngle.Append(radiansToDegrees(modules[3]->getDesiredAngle()));
    logBRActualAngle.Append(radiansToDegrees(actualStates[3].angle));
    logBRDriveOutput.Append(modules[3]->getDriveAppliedOutput());
    logBRSteerOutput.Append(modules[3]->getSteerAppliedOutput());
    logBRDriveCurrent.Append(modules[3]->getDriveOutputCurrent());
    logBRSteerCurrent.Append(modules[3]->getSteerOutputCurrent());
}
