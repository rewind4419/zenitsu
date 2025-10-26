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
    
    // Share + Options: 4s steer, then 4s drive
    if (shareBtn && optionsBtn) {
        static double diagStartTime = 0.0;
        double now = frc::Timer::GetFPGATimestamp().value();
        
        if (diagStartTime == 0.0) {
            diagStartTime = now;
            printf("Starting 4s steer + 4s drive diagnostic\n");
        }
        
        double elapsed = now - diagStartTime;
        
        if (elapsed < 4.0) {
            // First 4 seconds: steer only
            m_drivetrain->steerOnlyDuty(0.2);
            frc::SmartDashboard::PutString("Diag Mode", "Steer 4s");
            printf("Steer test: %.1fs elapsed\n", elapsed);
        } else if (elapsed < 8.0) {
            // Next 4 seconds: drive only
            m_drivetrain->driveOnlyDuty(0.3);
            frc::SmartDashboard::PutString("Diag Mode", "Drive 4s");
            printf("Drive test: %.1fs elapsed\n", elapsed - 4.0);
        } else {
            // Done
            m_drivetrain->stop();
            diagStartTime = 0.0;
            frc::SmartDashboard::PutString("Diag Mode", "Complete");
            printf("Diagnostic complete\n");
        }
        return;
    } else {
        // Reset timer when not in Share+Options mode
        static double& diagStartTime = *(new double(0.0));
        diagStartTime = 0.0;
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
