#include "GamepadInput.h"
#include "Config.h"

#include <frc/Timer.h>

GamepadInput::GamepadInput(int controllerPort) {
    m_controller = std::make_unique<frc::Joystick>(controllerPort);
}

void GamepadInput::update() {
    // Get raw stick values using PlayStation DualShock axis mapping
    double rawX = m_controller->GetRawAxis(0);      // Left stick X
    double rawY = -m_controller->GetRawAxis(1);     // Left stick Y (inverted)
    double rawRotation = m_controller->GetRawAxis(2); // Right stick X (PlayStation uses axis 2)
    
    // Apply deadband
    rawX = applyDeadband(rawX, JOYSTICK_DEADBAND);
    rawY = applyDeadband(rawY, JOYSTICK_DEADBAND);
    rawRotation = applyDeadband(rawRotation, JOYSTICK_DEADBAND);
    
    // Square inputs for better control (maintain sign)
    auto squareInput = [](double input) {
        return std::copysign(input * input, input);
    };
    
    m_driveTranslation.x = squareInput(rawY);  // Forward/backward
    m_driveTranslation.y = squareInput(rawX);  // Left/right
    m_rotation = squareInput(rawRotation);
    
    // Update button states using PlayStation DualShock button mapping
    m_precisionMode = m_controller->GetRawButton(5);  // L1 (left bumper)
    m_turboMode = m_controller->GetRawButton(6);      // R1 (right bumper)
    
    // Update PlayStation-specific buttons
    m_shareButton = m_controller->GetRawButton(9);    // Share button
    m_optionsButton = m_controller->GetRawButton(10); // Options button
    m_psButton = m_controller->GetRawButton(14);      // PlayStation button
    
    // Update safety timestamp
    m_lastControllerUpdate = frc::Timer::GetFPGATimestamp().value();
}

Vector2D GamepadInput::getDriveTranslation() const {
    double multiplier = getSpeedMultiplier();
    return m_driveTranslation * multiplier;
}

double GamepadInput::getRotation() const {
    double multiplier = getSpeedMultiplier();
    return m_rotation * multiplier;
}

bool GamepadInput::isPrecisionMode() const {
    return m_precisionMode;
}

bool GamepadInput::isTurboMode() const {
    return m_turboMode;
}

double GamepadInput::getSpeedMultiplier() const {
    if (m_precisionMode) {
        return PRECISION_DRIVE_SPEED;
    } else if (m_turboMode) {
        return TURBO_DRIVE_SPEED;
    } else {
        return NORMAL_DRIVE_SPEED;
    }
}

bool GamepadInput::getShareButton() const {
    return m_shareButton;
}

bool GamepadInput::getOptionsButton() const {
    return m_optionsButton;
}

bool GamepadInput::getPSButton() const {
    return m_psButton;
}

bool GamepadInput::isControllerConnected() const {
    double currentTime = frc::Timer::GetFPGATimestamp().value();
    return (currentTime - m_lastControllerUpdate) < CONTROLLER_TIMEOUT;
}
