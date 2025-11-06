#include "commands/TeleopDriveCommand.h"
#include "Config.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>

// Define NAVX_AVAILABLE based on header availability
#if __has_include(<Studica/AHRS.h>)
#include <Studica/AHRS.h>
#define NAVX_AVAILABLE 1
#else
#define NAVX_AVAILABLE 0
#endif


TeleopDriveCommand::TeleopDriveCommand(
    Drivetrain* drivetrain,
    GamepadInput* gamepadInput,
    void* navx,
    bool* fieldRelativePtr
) : m_drivetrain(drivetrain),
    m_gamepadInput(gamepadInput),
    m_navx(navx),
    m_fieldRelativePtr(fieldRelativePtr) {
    
    // Require the drivetrain subsystem
    AddRequirements({drivetrain});
}

void TeleopDriveCommand::Initialize() {
    // Reset slew rate limiter state
    m_prevVx = 0.0;
    m_prevVy = 0.0;
    m_prevOmega = 0.0;
    m_lastUpdateSec = frc::Timer::GetFPGATimestamp().value();
}

void TeleopDriveCommand::Execute() {
    // Safety checks
    bool controllerConnected = m_gamepadInput->isControllerConnected();
    
    // Stop if safety conditions not met
    if (!controllerConnected) {
        m_drivetrain->stop();
        return;
    }
    
    // Diagnostic modes (Options + L1/R1)
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
    
    // Compute target chassis speeds
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
    studica::AHRS* navx = static_cast<studica::AHRS*>(m_navx);
    
    // Debug: Show field-relative status
    frc::SmartDashboard::PutBoolean("Field Relative Enabled", *m_fieldRelativePtr);
    frc::SmartDashboard::PutBoolean("NavX Pointer Valid", navx != nullptr);
    
    // Check if the user WANTS field-relative and if the navx object exists.
    // We intentionally do NOT check navx->IsConnected() here because:
    // 1. The gyro can stream data while IsConnected() is still false (during calibration)
    // 2. If the gyro is calibrating, GetYaw() returns 0, which is safe
    // 3. The robot will use the correct heading as soon as calibration completes
    if (*m_fieldRelativePtr && navx) {
        // Use NavX for true field-relative driving (with yaw offset for physical mounting)
        double rawYaw = navx->GetYaw();
        double offsetYaw = rawYaw + NAVX_YAW_OFFSET_DEGREES;
        
        // Wrap angle to 0-360 range
        while (offsetYaw >= 360.0) offsetYaw -= 360.0;
        while (offsetYaw < 0.0) offsetYaw += 360.0;
        
        double gyroAngle = degreesToRadians(offsetYaw);
        
        // Debug: Show what angle we're using
        frc::SmartDashboard::PutNumber("Drive Mode Raw Yaw", rawYaw);
        frc::SmartDashboard::PutNumber("Drive Mode Offset Yaw", offsetYaw);
        frc::SmartDashboard::PutNumber("Drive Mode Gyro Angle (rad)", gyroAngle);
        frc::SmartDashboard::PutString("Drive Mode", "FIELD-RELATIVE");
        
        m_drivetrain->driveFieldRelative(speeds, gyroAngle);
    } else {
        // Robot-relative driving (field-relative is OFF or navx pointer is null)
        std::string reason = !(*m_fieldRelativePtr) ? "DISABLED" : "NO NAVX";
        frc::SmartDashboard::PutString("Drive Mode", "ROBOT-RELATIVE (" + reason + ")");
        m_drivetrain->drive(speeds);
    }
    #else
    // Robot-relative driving (NavX not available in this build)
    frc::SmartDashboard::PutString("Drive Mode", "ROBOT-RELATIVE (no NavX)");
    m_drivetrain->drive(speeds);
    #endif
    
    // Toggle field-relative mode with PS button (only when not in diagnostic mode)
    bool currentPSButton = m_gamepadInput->getPSButton();
    
    // Debug: Show button states
    frc::SmartDashboard::PutBoolean("PS Button Pressed", currentPSButton);
    frc::SmartDashboard::PutBoolean("PS Button Last State", m_lastPSButton);
    frc::SmartDashboard::PutBoolean("Options Button", optionsBtn);
    frc::SmartDashboard::PutBoolean("Share Button", shareBtn);
    
    if (currentPSButton && !m_lastPSButton && !optionsBtn && !shareBtn) {
        *m_fieldRelativePtr = !(*m_fieldRelativePtr);
        frc::SmartDashboard::PutBoolean("Field Relative", *m_fieldRelativePtr);
        printf("Field-relative mode %s\n", *m_fieldRelativePtr ? "ENABLED" : "DISABLED");
    }
    m_lastPSButton = currentPSButton;
}

void TeleopDriveCommand::End(bool interrupted) {
    // Stop the drivetrain when command ends
    m_drivetrain->stop();
}

bool TeleopDriveCommand::IsFinished() {
    // This command runs continuously during teleop
    return false;
}

