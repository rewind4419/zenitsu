#include "GamepadInput.h"
#include "Config.h"

#include <frc/Timer.h>

GamepadInput::GamepadInput(int controllerPort) {
    m_controller = std::make_unique<frc::Joystick>(controllerPort);
}

void GamepadInput::update() {
    // Get raw stick values using PlayStation DualShock axis mapping
    double rawX = m_controller->GetRawAxis(PS_AXIS_LEFT_X);
    double rawY = -m_controller->GetRawAxis(PS_AXIS_LEFT_Y);     // inverted
    double rawRotation = m_controller->GetRawAxis(PS_AXIS_RIGHT_X);
    
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
    m_precisionMode = m_controller->GetRawButton(PS_BTN_L1);
    m_turboMode = m_controller->GetRawButton(PS_BTN_R1);
    
    // Update PlayStation-specific buttons
    m_shareButton = m_controller->GetRawButton(PS_BTN_SHARE);
    m_optionsButton = m_controller->GetRawButton(PS_BTN_OPTIONS);
    m_psButton = m_controller->GetRawButton(PS_BTN_PS);
    
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
    if (m_turboMode) {
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
    return m_controller->IsConnected();
}
