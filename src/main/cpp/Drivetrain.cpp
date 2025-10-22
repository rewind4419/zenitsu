#include "Drivetrain.h"
#include "Config.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/velocity.h>
#include <units/angle.h>
#include <wpi/array.h>

Drivetrain::Drivetrain() {
    // Create swerve modules with their respective IDs and encoder offsets
    // Note: Encoder offsets would need to be calibrated for each robot
    m_modules[0] = std::make_unique<SwerveModule>(FRONT_LEFT_DRIVE_ID, FRONT_LEFT_STEER_ID, FRONT_LEFT_ENCODER_ID, FRONT_LEFT_ENCODER_OFFSET);
    m_modules[1] = std::make_unique<SwerveModule>(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_STEER_ID, FRONT_RIGHT_ENCODER_ID, FRONT_RIGHT_ENCODER_OFFSET);
    m_modules[2] = std::make_unique<SwerveModule>(BACK_LEFT_DRIVE_ID, BACK_LEFT_STEER_ID, BACK_LEFT_ENCODER_ID, BACK_LEFT_ENCODER_OFFSET);
    m_modules[3] = std::make_unique<SwerveModule>(BACK_RIGHT_DRIVE_ID, BACK_RIGHT_STEER_ID, BACK_RIGHT_ENCODER_ID, BACK_RIGHT_ENCODER_OFFSET);
    
    initializeModulePositions();

    // Build WPILib kinematics using module translations (FL, FR, BL, BR)
    m_kinematics = std::make_unique<frc::SwerveDriveKinematics<4>>(
        m_moduleTranslations[0],
        m_moduleTranslations[1],
        m_moduleTranslations[2],
        m_moduleTranslations[3]
    );

    // Initialize odometry at origin (guarded; can be used when gyro present)
    m_pose = frc::Pose2d{};
    m_odometry = std::make_unique<frc::SwerveDriveOdometry<4>>(
        *m_kinematics,
        frc::Rotation2d{units::radian_t{0.0}},
        getModulePositions(),
        m_pose
    );
}

void Drivetrain::driveFieldRelativeUnits(double vx, double vy, double omega, double yawRadians) {
    auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        units::meters_per_second_t{vx},
        units::meters_per_second_t{vy},
        units::radians_per_second_t{omega},
        frc::Rotation2d{units::radian_t{yawRadians}}
    );
    ChassisSpeed s;
    s.vx = speeds.vx.value();
    s.vy = speeds.vy.value();
    s.omega = speeds.omega.value();
    drive(s);
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

std::array<frc::SwerveModulePosition, 4> Drivetrain::getModulePositions() const {
    std::array<frc::SwerveModulePosition, 4> positions;
    for (size_t i = 0; i < 4; i++) {
        positions[i] = frc::SwerveModulePosition{
            units::meter_t{m_modules[i]->getDrivePosition()},
            frc::Rotation2d{units::radian_t{m_modules[i]->getCurrentState().angle}}
        };
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

void Drivetrain::resetOdometry(const frc::Pose2d& pose) {
    if (m_odometry == nullptr) return;
    m_pose = pose;
    m_odometry->ResetPosition(
        frc::Rotation2d{units::radian_t{0.0}},
        getModulePositions(),
        m_pose
    );
}

frc::Pose2d Drivetrain::updateOdometry(double gyroAngleRadians) {
    if (m_odometry == nullptr) return m_pose;
    m_pose = m_odometry->Update(
        frc::Rotation2d{units::radian_t{gyroAngleRadians}},
        getModulePositions()
    );
    return m_pose;
}

std::array<SwerveModuleState, 4> Drivetrain::calculateModuleStates(const ChassisSpeed& speeds) const {
    std::array<SwerveModuleState, 4> moduleStates;

    // Convert our ChassisSpeed to WPILib's for kinematics
    frc::ChassisSpeeds wpilibSpeeds{
        units::meters_per_second_t{speeds.vx},
        units::meters_per_second_t{speeds.vy},
        units::radians_per_second_t{speeds.omega}
    };
    auto wpilibStates = m_kinematics->ToSwerveModuleStates(wpilibSpeeds);

    for (size_t i = 0; i < 4; i++) {
        moduleStates[i].speed = wpilibStates[i].speed.value();
        moduleStates[i].angle = wpilibStates[i].angle.Radians().value();
    }

    return moduleStates;
}

void Drivetrain::normalizeWheelSpeeds(std::array<SwerveModuleState, 4>& moduleStates) const {
    // Use WPILib to desaturate speeds based on MAX_DRIVE_SPEED
    std::array<frc::SwerveModuleState, 4> stdStates;
    for (size_t i = 0; i < 4; i++) {
        stdStates[i] = frc::SwerveModuleState{
            units::meters_per_second_t{moduleStates[i].speed},
            frc::Rotation2d{units::radian_t{moduleStates[i].angle}}
        };
    }
    wpi::array<frc::SwerveModuleState, 4> wpiStates{stdStates};
    frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(&wpiStates, units::meters_per_second_t{MAX_DRIVE_SPEED});
    for (size_t i = 0; i < 4; i++) {
        moduleStates[i].speed = wpiStates[i].speed.value();
        moduleStates[i].angle = wpiStates[i].angle.Radians().value();
    }
}

void Drivetrain::initializeModulePositions() {
    // Module translations relative to robot center (WPILib: x forward, y left), meters
    const double halfLength = WHEELBASE_LENGTH / 2.0;
    const double halfWidth  = WHEELBASE_WIDTH  / 2.0;

    m_moduleTranslations[0] = frc::Translation2d{units::meter_t{ halfLength}, units::meter_t{ halfWidth}};  // FL
    m_moduleTranslations[1] = frc::Translation2d{units::meter_t{ halfLength}, units::meter_t{-halfWidth}};  // FR
    m_moduleTranslations[2] = frc::Translation2d{units::meter_t{-halfLength}, units::meter_t{ halfWidth}};  // BL
    m_moduleTranslations[3] = frc::Translation2d{units::meter_t{-halfLength}, units::meter_t{-halfWidth}};  // BR
}
