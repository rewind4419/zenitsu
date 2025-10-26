#pragma once

#include <rev/SparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <memory>

#include "MathUtils.h"

/**
 * Clean swerve module implementation
 * Represents a single swerve drive module (drive motor + steering motor + encoder)
 * Uses modern 2025 REV API
 */
class SwerveModule {
public:
    /**
     * Create a swerve module
     * @param driveMotorId CAN ID for drive motor (SparkMax)
     * @param steerMotorId CAN ID for steering motor (SparkMax)
     * @param encoderId CAN ID for absolute encoder (CANcoder)
     * @param encoderOffset Offset angle for absolute encoder (radians)
     */
    SwerveModule(int driveMotorId, int steerMotorId, int encoderId, double encoderOffset);
    
    /**
     * Set the desired state for this module
     * @param desiredState Target speed and angle
     */
    void setDesiredState(const SwerveModuleState& desiredState);
    
    /**
     * Get the current state of this module
     * @return Current speed and angle
     */
    SwerveModuleState getCurrentState() const;
    
    /**
     * Get the current position for odometry
     * @return Distance traveled by drive wheel (meters)
     */
    double getDrivePosition() const;
    
    /**
     * Reset the drive encoder to zero
     */
    void resetDriveEncoder();
    
    /**
     * Stop the module (set speed to 0, maintain current angle)
     */
    void stop();

    // Optional: expose desired angle for telemetry
    double getDesiredAngle() const { return m_desiredAngle; }
    
    // Telemetry: expose desired speed
    double getDesiredSpeed() const { return m_desiredSpeed; }

    /**
     * Raw absolute angle from CANcoder without offset, radians (0..2π)
     */
    double getRawAbsoluteAngle() const;
    
    // Telemetry helpers
    double getDriveAppliedOutput() const { return m_driveMotor ? m_driveMotor->GetAppliedOutput() : 0.0; }
    double getSteerAppliedOutput() const { return m_steerMotor ? m_steerMotor->GetAppliedOutput() : 0.0; }
    double getDriveOutputCurrent() const { return m_driveMotor ? m_driveMotor->GetOutputCurrent() : 0.0; }
    double getSteerOutputCurrent() const { return m_steerMotor ? m_steerMotor->GetOutputCurrent() : 0.0; }

    /**
     * Diagnostic: set drive motor duty directly
     */
    void setDriveOpenLoop(double duty);

    /**
     * Diagnostic: set steer motor duty directly
     */
    void setSteerOpenLoop(double duty);

private:
    // Hardware
    std::unique_ptr<rev::spark::SparkMax> m_driveMotor;
    std::unique_ptr<rev::spark::SparkMax> m_steerMotor;
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> m_encoder;
    
    // Configuration
    double m_encoderOffset;
    
    // State
    double m_lastAngle = 0.0;
    double m_desiredAngle = 0.0;
    double m_desiredSpeed = 0.0;
    
    // Helper functions
    double getAbsoluteAngle() const;
    SwerveModuleState optimizeState(const SwerveModuleState& desiredState, double currentAngle) const;
    void configureMotors();
};
