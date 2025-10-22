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

    /**
     * Raw absolute angle from CANcoder without offset, radians (0..2π)
     */
    double getRawAbsoluteAngle() const;

private:
    // Hardware
    std::unique_ptr<rev::spark::SparkMax> m_driveMotor;
    std::unique_ptr<rev::spark::SparkMax> m_steerMotor;
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> m_encoder;
    
    // Configuration
    double m_encoderOffset;
    
    // State
    double m_lastAngle = 0.0;
    
    // Helper functions
    double getAbsoluteAngle() const;
    SwerveModuleState optimizeState(const SwerveModuleState& desiredState, double currentAngle) const;
    void configureMotors();
};
