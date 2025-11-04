#include "commands/DiagnosticCommands.h"
#include <frc/smartdashboard/SmartDashboard.h>

// ============================================================================
// DriveOnlyCommand
// ============================================================================

DriveOnlyCommand::DriveOnlyCommand(Drivetrain* drivetrain, double duty)
    : m_drivetrain(drivetrain), m_duty(duty) {
    AddRequirements({drivetrain});
}

void DriveOnlyCommand::Initialize() {
    frc::SmartDashboard::PutString("Diag Mode", "Drive Only");
    printf("DriveOnlyCommand started: duty = %.2f\n", m_duty);
}

void DriveOnlyCommand::Execute() {
    m_drivetrain->driveOnlyDuty(m_duty);
}

void DriveOnlyCommand::End(bool interrupted) {
    m_drivetrain->stop();
    frc::SmartDashboard::PutString("Diag Mode", "Off");
    printf("DriveOnlyCommand ended\n");
}

bool DriveOnlyCommand::IsFinished() {
    return false;  // Runs until interrupted
}

// ============================================================================
// SteerOnlyCommand
// ============================================================================

SteerOnlyCommand::SteerOnlyCommand(Drivetrain* drivetrain, double duty)
    : m_drivetrain(drivetrain), m_duty(duty) {
    AddRequirements({drivetrain});
}

void SteerOnlyCommand::Initialize() {
    frc::SmartDashboard::PutString("Diag Mode", "Steer Only");
    printf("SteerOnlyCommand started: duty = %.2f\n", m_duty);
}

void SteerOnlyCommand::Execute() {
    m_drivetrain->steerOnlyDuty(m_duty);
}

void SteerOnlyCommand::End(bool interrupted) {
    m_drivetrain->stop();
    frc::SmartDashboard::PutString("Diag Mode", "Off");
    printf("SteerOnlyCommand ended\n");
}

bool SteerOnlyCommand::IsFinished() {
    return false;  // Runs until interrupted
}

