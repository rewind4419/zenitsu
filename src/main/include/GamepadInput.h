#pragma once

#include <frc/Joystick.h>
#include <memory>

#include "MathUtils.h"

/**
 * Clean PlayStation controller input handling
 * Manages driver controller input with proper deadbands and scaling
 * Uses PlayStation DualShock controller layout
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
     * Check if precision mode is active (L1 bumper)
     * @return true if precision mode should be enabled
     */
    bool isPrecisionMode() const;
    
    /**
     * Check if turbo mode is active (R1 bumper)
     * @return true if turbo mode should be enabled
     */
    bool isTurboMode() const;
    
    /**
     * Check PlayStation-specific buttons
     */
    bool getShareButton() const;
    bool getOptionsButton() const;
    bool getPSButton() const;
    
    /**
     * Get the current speed multiplier based on active modes
     * @return Speed multiplier (0.0 to 1.0)
     */
    double getSpeedMultiplier() const;

private:
    std::unique_ptr<frc::Joystick> m_controller;
    
    // Input state
    Vector2D m_driveTranslation;
    double m_rotation = 0.0;
    bool m_precisionMode = false;
    bool m_turboMode = false;
    
    // PlayStation button state
    bool m_shareButton = false;
    bool m_optionsButton = false; 
    bool m_psButton = false;
};
