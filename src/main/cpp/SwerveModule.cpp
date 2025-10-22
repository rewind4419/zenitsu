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
    double currentAngle = getAbsoluteAngle();
    
    // Optimize the state to avoid spinning more than 90 degrees
    SwerveModuleState optimizedState = optimizeState(desiredState, currentAngle);
    
    // Set drive motor speed
    double driveOutput = optimizedState.speed / MAX_DRIVE_SPEED;  // Normalize to -1 to 1
    m_driveMotor->Set(driveOutput);
    
    // Set steering motor position using PID controller
    auto steerPID = m_steerMotor->GetClosedLoopController();
    steerPID.SetReference(optimizedState.angle, rev::spark::SparkMax::ControlType::kPosition);
    
    m_lastAngle = optimizedState.angle;
}

SwerveModuleState SwerveModule::getCurrentState() const {
    return {
        .speed = m_driveMotor->GetEncoder().GetVelocity() * WHEEL_RADIUS * 2 * M_PI / 60.0 / DRIVE_GEAR_RATIO,
        .angle = getAbsoluteAngle()
    };
}

double SwerveModule::getDrivePosition() const {
    // Convert motor rotations to wheel distance
    double motorRotations = m_driveMotor->GetEncoder().GetPosition();
    double wheelRotations = motorRotations / DRIVE_GEAR_RATIO;
    return wheelRotations * WHEEL_RADIUS * 2 * M_PI;
}

void SwerveModule::resetDriveEncoder() {
    m_driveMotor->GetEncoder().SetPosition(0.0);
}

void SwerveModule::stop() {
    m_driveMotor->Set(0.0);
    // Keep steering at current position
}

double SwerveModule::getAbsoluteAngle() const {
    // Get absolute encoder value and apply offset
    double rawAngle = m_encoder->GetAbsolutePosition().GetValue().value() * 2 * M_PI;
    return constrainAngle(rawAngle - m_encoderOffset);
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
    // Configure drive motor
    m_driveMotor->RestoreFactoryDefaults();
    m_driveMotor->SetIdleMode(rev::spark::SparkMax::IdleMode::kBrake);
    m_driveMotor->SetSmartCurrentLimit(40);  // 40A limit
    m_driveMotor->GetEncoder().SetPosition(0.0);
    m_driveMotor->GetEncoder().SetPositionConversionFactor(WHEEL_RADIUS * 2 * M_PI / DRIVE_GEAR_RATIO);
    m_driveMotor->GetEncoder().SetVelocityConversionFactor(WHEEL_RADIUS * 2 * M_PI / DRIVE_GEAR_RATIO / 60.0);
    
    // Configure steering motor for position control
    m_steerMotor->RestoreFactoryDefaults();
    m_steerMotor->SetIdleMode(rev::spark::SparkMax::IdleMode::kBrake);
    m_steerMotor->SetSmartCurrentLimit(20);  // 20A limit for steering
    
    // Configure steering PID controller
    auto steerPID = m_steerMotor->GetClosedLoopController();
    steerPID.SetP(0.1);   // Start with conservative PID values
    steerPID.SetI(0.0);
    steerPID.SetD(0.0);
    steerPID.SetFF(0.0);
    steerPID.SetOutputRange(-1.0, 1.0);
    
    // Set steering encoder conversion factor (rotations -> radians)
    m_steerMotor->GetEncoder().SetPositionConversionFactor(2 * M_PI / STEER_GEAR_RATIO);
    m_steerMotor->GetEncoder().SetPosition(0.0);
    
    // Burn flash to save configuration
    m_driveMotor->BurnFlash();
    m_steerMotor->BurnFlash();
}
