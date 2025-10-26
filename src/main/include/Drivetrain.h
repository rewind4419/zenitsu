#pragma once

#include <array>
#include <memory>

#include "SwerveModule.h"
#include "MathUtils.h"

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

/**
 * Clean swerve drivetrain implementation
 * Manages 4 swerve modules and provides high-level drive control
 */
class Drivetrain {
public:
    Drivetrain();
    
    /**
     * Drive the robot with chassis-relative speeds
     * @param speeds Desired forward, sideways, and rotational velocities
     */
    void drive(const ChassisSpeed& speeds);
    
    /**
     * Drive the robot with field-relative speeds
     * @param speeds Desired field-relative velocities  
     * @param gyroAngle Current robot heading (radians)
     */
    void driveFieldRelative(const ChassisSpeed& speeds, double gyroAngle);
    
    /**
     * Convenience: field-relative drive using unitless doubles (m/s, rad/s, yaw radians)
     */
    void driveFieldRelativeUnits(double vx, double vy, double omega, double yawRadians);
    
    /**
     * Stop all modules
     */
    void stop();
    
    /**
     * Get current module states (for odometry/debugging)
     * @return Array of current module states [FL, FR, BL, BR]
     */
    std::array<SwerveModuleState, 4> getModuleStates() const;
    
    /**
     * Get current wheel positions (for odometry)  
     * @return Array of drive wheel positions in meters [FL, FR, BL, BR]
     */
    std::array<double, 4> getWheelPositions() const;

    /**
     * Get module positions for odometry
     */
    std::array<frc::SwerveModulePosition, 4> getModulePositions() const;

    /**
     * Raw absolute angles from CANcoders (no offsets), radians
     */
    std::array<double, 4> getRawModuleAngles() const;
    
    /**
     * Reset drive encoders to zero
     */
    void resetEncoders();
    
    /**
     * Update telemetry/debugging info
     */
    void updateTelemetry();

    /**
     * Odometry helpers
     */
    void resetOdometry(const frc::Pose2d& pose);
    frc::Pose2d updateOdometry(double gyroAngleRadians);
    frc::Pose2d getPose() const { return m_pose; }

    /**
     * Diagnostic: drive all motors at fixed duty (drive only, steer = 0)
     */
    void driveOnlyDuty(double duty);

    /**
     * Diagnostic: steer all motors at fixed duty (steer only, drive = 0)
     */
    void steerOnlyDuty(double duty);
    
    /**
     * Get last commanded chassis speeds for telemetry
     */
    ChassisSpeed getLastCommandedSpeeds() const { return m_lastCommandedSpeeds; }
    
    /**
     * Get direct access to modules for detailed telemetry
     */
    const std::array<std::unique_ptr<SwerveModule>, 4>& getModules() const { return m_modules; }

private:
    // Swerve modules: Front-Left, Front-Right, Back-Left, Back-Right
    std::array<std::unique_ptr<SwerveModule>, 4> m_modules;
    
    // WPILib swerve kinematics
    std::unique_ptr<frc::SwerveDriveKinematics<4>> m_kinematics;
    std::array<frc::Translation2d, 4> m_moduleTranslations; // FL, FR, BL, BR

    // WPILib odometry
    std::unique_ptr<frc::SwerveDriveOdometry<4>> m_odometry;
    frc::Pose2d m_pose;
    
    // Last commanded speeds for telemetry
    ChassisSpeed m_lastCommandedSpeeds;
    
    /**
     * Convert chassis speeds to individual module states
     * @param speeds Desired robot-relative velocities
     * @return Array of target module states [FL, FR, BL, BR] 
     */
    std::array<SwerveModuleState, 4> calculateModuleStates(const ChassisSpeed& speeds) const;
    
    /**
     * Normalize wheel speeds so none exceed the maximum
     * @param moduleStates Array of module states to normalize
     */
    void normalizeWheelSpeeds(std::array<SwerveModuleState, 4>& moduleStates) const;
    
    /**
     * Initialize module positions based on robot geometry
     */
    void initializeModulePositions();
};
