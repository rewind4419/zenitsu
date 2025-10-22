#include "Drivetrain.h"
#include "Config.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>

Drivetrain::Drivetrain() {
    // Create swerve modules with their respective IDs and encoder offsets
    // Note: Encoder offsets would need to be calibrated for each robot
    m_modules[0] = std::make_unique<SwerveModule>(FRONT_LEFT_DRIVE_ID, FRONT_LEFT_STEER_ID, FRONT_LEFT_ENCODER_ID, FRONT_LEFT_ENCODER_OFFSET);
    m_modules[1] = std::make_unique<SwerveModule>(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_STEER_ID, FRONT_RIGHT_ENCODER_ID, FRONT_RIGHT_ENCODER_OFFSET);
    m_modules[2] = std::make_unique<SwerveModule>(BACK_LEFT_DRIVE_ID, BACK_LEFT_STEER_ID, BACK_LEFT_ENCODER_ID, BACK_LEFT_ENCODER_OFFSET);
    m_modules[3] = std::make_unique<SwerveModule>(BACK_RIGHT_DRIVE_ID, BACK_RIGHT_STEER_ID, BACK_RIGHT_ENCODER_ID, BACK_RIGHT_ENCODER_OFFSET);
    
    initializeModulePositions();
}

void Drivetrain::drive(const ChassisSpeed& speeds) {
    auto moduleStates = calculateModuleStates(speeds);
    normalizeWheelSpeeds(moduleStates);
    
    for (size_t i = 0; i < 4; i++) {
        m_modules[i]->setDesiredState(moduleStates[i]);
    }
}

void Drivetrain::driveFieldRelative(const ChassisSpeed& speeds, double gyroAngle) {
    // Convert field-relative speeds to robot-relative
    ChassisSpeed robotRelative;
    robotRelative.vx = speeds.vx * std::cos(gyroAngle) + speeds.vy * std::sin(gyroAngle);
    robotRelative.vy = -speeds.vx * std::sin(gyroAngle) + speeds.vy * std::cos(gyroAngle);
    robotRelative.omega = speeds.omega;
    
    drive(robotRelative);
}

void Drivetrain::stop() {
    for (auto& module : m_modules) {
        module->stop();
    }
}

std::array<SwerveModuleState, 4> Drivetrain::getModuleStates() const {
    std::array<SwerveModuleState, 4> states;
    for (size_t i = 0; i < 4; i++) {
        states[i] = m_modules[i]->getCurrentState();
    }
    return states;
}

std::array<double, 4> Drivetrain::getWheelPositions() const {
    std::array<double, 4> positions;
    for (size_t i = 0; i < 4; i++) {
        positions[i] = m_modules[i]->getDrivePosition();
    }
    return positions;
}

std::array<double, 4> Drivetrain::getRawModuleAngles() const {
    std::array<double, 4> angles;
    for (size_t i = 0; i < 4; i++) {
        angles[i] = m_modules[i]->getRawAbsoluteAngle();
    }
    return angles;
}

void Drivetrain::resetEncoders() {
    for (auto& module : m_modules) {
        module->resetDriveEncoder();
    }
}

void Drivetrain::updateTelemetry() {
    auto states = getModuleStates();
    
    frc::SmartDashboard::PutNumber("FL Speed", states[0].speed);
    frc::SmartDashboard::PutNumber("FR Speed", states[1].speed);
    frc::SmartDashboard::PutNumber("BL Speed", states[2].speed);
    frc::SmartDashboard::PutNumber("BR Speed", states[3].speed);
    
    frc::SmartDashboard::PutNumber("FL Angle", radiansToDegrees(states[0].angle));
    frc::SmartDashboard::PutNumber("FR Angle", radiansToDegrees(states[1].angle));
    frc::SmartDashboard::PutNumber("BL Angle", radiansToDegrees(states[2].angle));
    frc::SmartDashboard::PutNumber("BR Angle", radiansToDegrees(states[3].angle));
}

std::array<SwerveModuleState, 4> Drivetrain::calculateModuleStates(const ChassisSpeed& speeds) const {
    std::array<SwerveModuleState, 4> moduleStates;
    
    for (size_t i = 0; i < 4; i++) {
        // Calculate the velocity vector for this module
        Vector2D moduleVelocity;
        moduleVelocity.x = speeds.vx - speeds.omega * m_modulePositions[i].position.y;
        moduleVelocity.y = speeds.vy + speeds.omega * m_modulePositions[i].position.x;
        
        // Convert to speed and angle
        moduleStates[i].speed = moduleVelocity.magnitude();
        moduleStates[i].angle = std::atan2(moduleVelocity.y, moduleVelocity.x);
    }
    
    return moduleStates;
}

void Drivetrain::normalizeWheelSpeeds(std::array<SwerveModuleState, 4>& moduleStates) const {
    // Find the maximum speed
    double maxSpeed = 0.0;
    for (const auto& state : moduleStates) {
        maxSpeed = std::max(maxSpeed, std::abs(state.speed));
    }
    
    // If any speed exceeds maximum, scale all speeds down proportionally
    if (maxSpeed > MAX_DRIVE_SPEED) {
        double scale = MAX_DRIVE_SPEED / maxSpeed;
        for (auto& state : moduleStates) {
            state.speed *= scale;
        }
    }
}

void Drivetrain::initializeModulePositions() {
    // Module positions relative to robot center (x forward, y left)
    double halfLength = WHEELBASE_LENGTH / 2.0;
    double halfWidth = WHEELBASE_WIDTH / 2.0;
    
    m_modulePositions[0].position = {halfLength, halfWidth};   // Front Left
    m_modulePositions[1].position = {halfLength, -halfWidth};  // Front Right  
    m_modulePositions[2].position = {-halfLength, halfWidth};  // Back Left
    m_modulePositions[3].position = {-halfLength, -halfWidth}; // Back Right
}
