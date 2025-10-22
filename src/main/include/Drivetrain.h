#pragma once

#include <array>
#include <memory>

#include "SwerveModule.h"
#include "MathUtils.h"

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

private:
    // Swerve modules: Front-Left, Front-Right, Back-Left, Back-Right
    std::array<std::unique_ptr<SwerveModule>, 4> m_modules;
    
    // Swerve kinematics
    struct ModulePosition {
        Vector2D position;  // Position relative to robot center
    };
    
    std::array<ModulePosition, 4> m_modulePositions;
    
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
