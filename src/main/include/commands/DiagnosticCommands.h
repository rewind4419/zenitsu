#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Drivetrain.h"

/**
 * Drive-only diagnostic command
 * Runs all drive motors at a fixed duty cycle with steer motors off
 */
class DriveOnlyCommand : public frc2::CommandHelper<frc2::Command, DriveOnlyCommand> {
public:
    explicit DriveOnlyCommand(Drivetrain* drivetrain, double duty = 0.3);
    
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    Drivetrain* m_drivetrain;
    double m_duty;
};

/**
 * Steer-only diagnostic command
 * Runs all steer motors at a fixed duty cycle with drive motors off
 */
class SteerOnlyCommand : public frc2::CommandHelper<frc2::Command, SteerOnlyCommand> {
public:
    explicit SteerOnlyCommand(Drivetrain* drivetrain, double duty = 0.2);
    
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    Drivetrain* m_drivetrain;
    double m_duty;
};

