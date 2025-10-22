#pragma once

#include <frc/XboxController.h>
#include <memory>

#include "MathUtils.h"

/**
 * Clean gamepad input handling
 * Manages driver controller input with proper deadbands and scaling
 */
class GamepadInput {
public:
    GamepadInput(int controllerPort);
    
    /**
     * Update input values (call this every loop)
     */
    void update();
    
    /**
     * Get drive translation input (left stick)
     * @return Forward/sideways input with deadband applied
     */
    Vector2D getDriveTranslation() const;
    
    /**
     * Get rotation input (right stick X)
     * @return Rotational input with deadband applied
     */
    double getRotation() const;
    
    /**
     * Check if precision mode is active (left bumper)
     * @return true if precision mode should be enabled
     */
    bool isPrecisionMode() const;
    
    /**
     * Check if turbo mode is active (right bumper)
     * @return true if turbo mode should be enabled
     */
    bool isTurboMode() const;
    
    /**
     * Get the current speed multiplier based on active modes
     * @return Speed multiplier (0.0 to 1.0)
     */
    double getSpeedMultiplier() const;

private:
    std::unique_ptr<frc::XboxController> m_controller;
    
    // Input state
    Vector2D m_driveTranslation;
    double m_rotation = 0.0;
    bool m_precisionMode = false;
    bool m_turboMode = false;
};
