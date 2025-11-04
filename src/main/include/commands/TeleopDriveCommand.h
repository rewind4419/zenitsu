#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "Drivetrain.h"
#include "GamepadInput.h"
#include "Config.h"

#if __has_include(<Studica/AHRS.h>)
#include <Studica/AHRS.h>
#endif

/**
 * Teleop drive command - handles driver input and controls the drivetrain
 * This is the default command for the drivetrain subsystem during teleop
 */
class TeleopDriveCommand : public frc2::CommandHelper<frc2::Command, TeleopDriveCommand> {
public:
    /**
     * Constructor
     * @param drivetrain Pointer to drivetrain subsystem
     * @param gamepadInput Pointer to gamepad input handler
     * @param navx Pointer to NavX gyroscope (can be nullptr if not available)
     * @param fieldRelativePtr Pointer to field-relative mode flag
     */
    TeleopDriveCommand(
        Drivetrain* drivetrain,
        GamepadInput* gamepadInput,
        void* navx,  // void* to avoid conditional compilation issues
        bool* fieldRelativePtr
    );

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    Drivetrain* m_drivetrain;
    GamepadInput* m_gamepadInput;
    void* m_navx;  // Cast to studica::AHRS* when needed
    bool* m_fieldRelativePtr;
    
    // Slew rate limiter state
    double m_prevVx = 0.0;
    double m_prevVy = 0.0;
    double m_prevOmega = 0.0;
    double m_lastUpdateSec = 0.0;
    
    // Button state tracking for toggles
    bool m_lastPSButton = false;
};

