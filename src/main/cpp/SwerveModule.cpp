#include "SwerveModule.h"
#include "Config.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace rev::spark;

SwerveModule::SwerveModule(int driveMotorId, int steerMotorId, int encoderId, double encoderOffset)
    : m_encoderOffset(encoderOffset) {
    
    // Create motors with 2025 API
    m_driveMotor = std::make_unique<SparkMax>(driveMotorId, SparkMax::MotorType::kBrushless);
    m_steerMotor = std::make_unique<SparkMax>(steerMotorId, SparkMax::MotorType::kBrushless);
    m_encoder = std::make_unique<ctre::phoenix6::hardware::CANcoder>(encoderId);
    
    configureMotors();
}

void SwerveModule::setDesiredState(const SwerveModuleState& desiredState) {
    // Get current angle
    const double currentAngle = getAbsoluteAngle();

    // Optimize the state to avoid spinning more than 90 degrees
    const SwerveModuleState optimizedState = optimizeState(desiredState, currentAngle);
    m_desiredAngle = optimizedState.angle;
    m_desiredSpeed = optimizedState.speed;

    // Drive output normalized -1..1
    const double driveOutput = optimizedState.speed / MAX_DRIVE_SPEED;
    m_driveMotor->Set(driveOutput);

    // Steering: prefer onboard position PID if available, else simple P
    #if USE_ONBOARD_STEER_PID
    {
        // If steer encoder is set to radians via conversion factor, target is radians
        // Otherwise convert to motor rotations: angle(rad) * STEER_GEAR_RATIO / (2π)
        double targetPosition = optimizedState.angle;
        #ifndef REV_BRUSHLESS
        targetPosition = optimizedState.angle / (2.0 * M_PI) * STEER_GEAR_RATIO;
        #endif
        auto ctrl = m_steerMotor->GetClosedLoopController();
        // Live kP tuning from dashboard (CONFIGURABLE) — if API lacks SetP here, keep default
        double kP = frc::SmartDashboard::GetNumber("Steer kP", STEER_P_GAIN);
        (void)kP; // avoid unused if controller doesn't expose SetP in this build
        ctrl.SetReference(targetPosition, rev::spark::SparkMax::ControlType::kPosition);
    }
    #else
    {
        double angleError = constrainAngle(optimizedState.angle - currentAngle);
        double kP = frc::SmartDashboard::GetNumber("Steer kP", STEER_P_GAIN);
        double steerCmd = kP * angleError; // unitless output
        if (steerCmd > 1.0) steerCmd = 1.0;
        if (steerCmd < -1.0) steerCmd = -1.0;
        m_steerMotor->Set(steerCmd);
    }
    #endif

    m_lastAngle = optimizedState.angle;
}

SwerveModuleState SwerveModule::getCurrentState() const {
    #ifdef REV_BRUSHLESS
    // Encoder configured to report m/s directly
    return {
        .speed = m_driveMotor->GetEncoder().GetVelocity(),
        .angle = getAbsoluteAngle()
    };
    #else
    // Fallback for CI/sim: compute m/s from RPM
    return {
        .speed = m_driveMotor->GetEncoder().GetVelocity() * WHEEL_RADIUS * 2 * M_PI / 60.0 / DRIVE_GEAR_RATIO,
        .angle = getAbsoluteAngle()
    };
    #endif
}

double SwerveModule::getDrivePosition() const {
    #ifdef REV_BRUSHLESS
    // Encoder configured to report meters directly
    return m_driveMotor->GetEncoder().GetPosition();
    #else
    // Fallback for CI/sim: convert motor rotations to meters
    double motorRotations = m_driveMotor->GetEncoder().GetPosition();
    double wheelRotations = motorRotations / DRIVE_GEAR_RATIO;
    return wheelRotations * WHEEL_RADIUS * 2 * M_PI;
    #endif
}

void SwerveModule::resetDriveEncoder() {
    m_driveMotor->GetEncoder().SetPosition(0.0);
}

void SwerveModule::stop() {
    m_driveMotor->Set(0.0);
    // Keep steering at current position
}

void SwerveModule::setDriveOpenLoop(double duty) {
    if (!std::isfinite(duty)) duty = 0.0;
    if (duty > 1.0) duty = 1.0;
    if (duty < -1.0) duty = -1.0;
    m_driveMotor->Set(duty);
}

void SwerveModule::setSteerOpenLoop(double duty) {
    if (!std::isfinite(duty)) duty = 0.0;
    if (duty > 1.0) duty = 1.0;
    if (duty < -1.0) duty = -1.0;
    m_steerMotor->Set(duty);
}

double SwerveModule::getAbsoluteAngle() const {
    // Get absolute encoder value and apply offset
    double rawAngle = m_encoder->GetAbsolutePosition().GetValue().value() * 2 * M_PI;
    return constrainAngle(rawAngle - m_encoderOffset);
}

double SwerveModule::getRawAbsoluteAngle() const {
    return m_encoder->GetAbsolutePosition().GetValue().value() * 2.0 * M_PI;
}

SwerveModuleState SwerveModule::optimizeState(const SwerveModuleState& desiredState, double currentAngle) const {
    double targetAngle = constrainAngle(desiredState.angle);
    double delta = constrainAngle(targetAngle - currentAngle);
    
    // If we need to turn more than 90 degrees, reverse the wheel and turn the other way
    if (std::abs(delta) > M_PI / 2) {
        return {
            .speed = -desiredState.speed,
            .angle = constrainAngle(targetAngle - M_PI)
        };
    } else {
        return {
            .speed = desiredState.speed,
            .angle = targetAngle
        };
    }
}

void SwerveModule::configureMotors() {
    // Idle mode and current limits
    // NOTE for students:
    // We wrap the REV-specific configuration in a preprocessor guard so CI/simulation
    // builds (which may not have the REV vendor library available) still compile.
    // On the real robot image, the REV library is present and these lines will run,
    // enabling brake mode (modules hold position when you release sticks) and current
    // limits (protects breakers and prevents brownouts). If you always have the REV
    // vendordep installed in your environment, you can instead use
    //   #if __has_include(<rev/SparkMax.h>)
    // as the guard condition.
    // These settings are essential for a controllable swerve drivetrain on the robot.
    // Apply brake mode and current limits where APIs are available
    #ifdef REV_BRUSHLESS
    m_driveMotor->SetIdleMode(rev::spark::SparkMax::IdleMode::kBrake);
    m_steerMotor->SetIdleMode(rev::spark::SparkMax::IdleMode::kBrake);
    m_driveMotor->SetSmartCurrentLimit(40);
    m_steerMotor->SetSmartCurrentLimit(20);
    #endif

    // Conversion factors for student-friendly units
    const double driveCF = (WHEEL_RADIUS * 2.0 * M_PI) / DRIVE_GEAR_RATIO; // rotations → meters
    // Enable conversion factors when available (lets encoders report meters / m/s / radians)
    #ifdef REV_BRUSHLESS
    m_driveMotor->GetEncoder().SetPositionConversionFactor(driveCF);
    m_driveMotor->GetEncoder().SetVelocityConversionFactor(driveCF / 60.0);
    #endif

    const double steerCF = (2.0 * M_PI) / STEER_GEAR_RATIO; // rotations → radians
    #ifdef REV_BRUSHLESS
    m_steerMotor->GetEncoder().SetPositionConversionFactor(steerCF);
    #endif

    // Seed steer encoder to current absolute angle (with offset)
    m_steerMotor->GetEncoder().SetPosition(getAbsoluteAngle());
}
