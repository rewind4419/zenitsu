#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/TimedRobot.h>
#include <frc/filter/SlewRateLimiter.h>  // Input smoothing (tune rates per team preference)
#include <frc/smartdashboard/Field2d.h> // Visualize odometry on dashboard
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>

#if __has_include(<Studica/AHRS.h>)
#include <Studica/AHRS.h>
#define NAVX_AVAILABLE 1
#else
#define NAVX_AVAILABLE 0
#endif
#include <memory>

#include "Drivetrain.h"
#include "GamepadInput.h"
#include "VisionSubsystem.h"
#include "commands/TeleopDriveCommand.h"
#include "commands/DiagnosticCommands.h"
#include "commands/DriveToAprilTagCommand.h"

/**
 * Command-based swerve robot implementation
 * Uses TimedRobot with CommandScheduler for subsystem management
 */
class Robot : public frc::TimedRobot {
public:
    Robot();
    
    void RobotInit() override;
    void RobotPeriodic() override;
    
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    
    void TeleopInit() override;
    void TeleopPeriodic() override;
    
    void DisabledInit() override;
    void DisabledPeriodic() override;

private:
    // Hardware
    std::unique_ptr<Drivetrain> m_drivetrain;
    std::unique_ptr<GamepadInput> m_gamepadInput;
    std::unique_ptr<VisionSubsystem> m_vision;
    #if NAVX_AVAILABLE
    std::unique_ptr<studica::AHRS> m_navx;
    #endif
    
    // Robot state
    bool m_fieldRelative = true;  // Start in field-relative mode
    
    // Autonomous timer
    double m_autoStartTime = 0.0;

    // Field visualization (odometry)
    frc::Field2d m_field;
    
    // Commands
    std::unique_ptr<TeleopDriveCommand> m_teleopDriveCommand;
    std::unique_ptr<DriveToAprilTagCommand> m_autoCommand;
    
    /**
     * Update dashboard values  
     */
    void updateDashboard();
    
    /**
     * Log drivetrain performance data to DataLog for AdvantageScope analysis
     */
    void logDrivetrainPerformance();
    
    /**
     * Update pose estimation with vision measurements
     * Fuses AprilTag detections with odometry using Kalman filter
     */
    void updateVisionPoseEstimation();
};
