#pragma once

#include <frc/TimedRobot.h>
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
    
    // Robot state
    bool m_fieldRelative = true;  // Start in field-relative mode
    
    /**
     * Handle teleop driving
     */
    void handleTeleopDrive();
    
    /**
     * Update dashboard values  
     */
    void updateDashboard();
};
