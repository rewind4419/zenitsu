#pragma once

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

/**
 * Clean swerve robot implementation
 * Simple, focused robot class for drivetrain-only operation
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
    #if NAVX_AVAILABLE
    std::unique_ptr<studica::AHRS> m_navx;
    #endif
    
    // Robot state
    bool m_fieldRelative = true;  // Start in field-relative mode

    // Driver input slew rate limiters (CONFIGURABLE: tune for driver feel)
    // Units: how fast the command can change (per second)
    // WPILib SlewRateLimiter in 2025 uses units. We keep doubles here for clarity and
    // apply the rates manually in code to avoid template friction for students.
    double m_prevVx = 0.0;
    double m_prevVy = 0.0;
    double m_prevOmega = 0.0;
    double m_lastUpdateSec = 0.0;
    
    // Autonomous timer
    double m_autoStartTime = 0.0;

    // Field visualization (odometry)
    frc::Field2d m_field;
    
    /**
     * Handle teleop driving
     */
    void handleTeleopDrive();
    
    /**
     * Update dashboard values  
     */
    void updateDashboard();
    
    /**
     * Log drivetrain performance data to DataLog for AdvantageScope analysis
     */
    void logDrivetrainPerformance();
};
