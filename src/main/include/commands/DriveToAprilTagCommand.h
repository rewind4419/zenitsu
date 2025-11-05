#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Drivetrain.h"
#include "VisionSubsystem.h"

/**
 * Simple autonomous command to drive toward an AprilTag
 * 
 * This is a basic example for testing vision-based navigation.
 * For production use, consider using PathPlanner or similar for more sophisticated paths.
 * 
 * Behavior:
 * - Continuously looks for AprilTags
 * - Drives toward the detected tag at reduced speed
 * - Stops when within target distance
 * - Times out after specified duration
 */
class DriveToAprilTagCommand : public frc2::CommandHelper<frc2::Command, DriveToAprilTagCommand> {
public:
    /**
     * Constructor
     * @param drivetrain Pointer to drivetrain subsystem
     * @param vision Pointer to vision subsystem
     * @param targetDistance Target distance from tag in meters (default 1.0m)
     * @param timeoutSeconds Maximum time to run command (default 5.0s)
     */
    DriveToAprilTagCommand(
        Drivetrain* drivetrain,
        VisionSubsystem* vision,
        double targetDistance = 1.0,
        double timeoutSeconds = 5.0
    );

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    Drivetrain* m_drivetrain;
    VisionSubsystem* m_vision;
    double m_targetDistance;
    double m_timeoutSeconds;
    double m_startTime;
    
    // Simple P controller gains
    static constexpr double kP_Forward = 0.5;   // Forward speed proportional gain
    static constexpr double kP_Rotation = 2.0;  // Rotation proportional gain
    static constexpr double kMaxSpeed = 0.5;    // Max speed (m/s)
    static constexpr double kDistanceTolerance = 0.1;  // Distance tolerance (m)
};

