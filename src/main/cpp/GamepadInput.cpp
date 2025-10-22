#include "GamepadInput.h"
#include "Config.h"

GamepadInput::GamepadInput(int controllerPort) {
    m_controller = std::make_unique<frc::XboxController>(controllerPort);
}

void GamepadInput::update() {
    // Get raw stick values
    double rawX = m_controller->GetLeftX();
    double rawY = -m_controller->GetLeftY();  // Invert Y axis (up is positive)
    double rawRotation = m_controller->GetRightX();
    
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
    
    // Update button states
    m_precisionMode = m_controller->GetLeftBumper();
    m_turboMode = m_controller->GetRightBumper();
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
