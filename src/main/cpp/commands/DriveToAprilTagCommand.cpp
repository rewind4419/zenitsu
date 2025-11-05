#include "commands/DriveToAprilTagCommand.h"
#include "Config.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>

DriveToAprilTagCommand::DriveToAprilTagCommand(
    Drivetrain* drivetrain,
    VisionSubsystem* vision,
    double targetDistance,
    double timeoutSeconds
) : m_drivetrain(drivetrain),
    m_vision(vision),
    m_targetDistance(targetDistance),
    m_timeoutSeconds(timeoutSeconds),
    m_startTime(0.0) {
    
    AddRequirements({drivetrain});
}

void DriveToAprilTagCommand::Initialize() {
    m_startTime = frc::Timer::GetFPGATimestamp().value();
    printf("DriveToAprilTagCommand started - target distance: %.2fm\n", m_targetDistance);
    frc::SmartDashboard::PutString("Auto Status", "Driving to AprilTag");
}

void DriveToAprilTagCommand::Execute() {
    // Check if we can see any AprilTags
    if (!m_vision->hasTargets()) {
        // No tags visible - stop and wait
        m_drivetrain->stop();
        frc::SmartDashboard::PutString("Auto Status", "Searching for tags...");
        return;
    }
    
    // Get vision pose estimate
    auto visionPoseEstimate = m_vision->getEstimatedGlobalPose();
    if (!visionPoseEstimate.has_value()) {
        // Can't get pose - stop
        m_drivetrain->stop();
        return;
    }
    
    // Get the latest camera result to find tag distance/angle
    auto result = m_vision->getLatestResult();
    if (!result.HasTargets()) {
        m_drivetrain->stop();
        return;
    }
    
    // Get best target
    auto bestTarget = result.GetBestTarget();
    
    // Simple approach: use tag's yaw and pitch to estimate distance
    // For a more accurate approach, you'd use the full 3D transform
    double yaw = bestTarget.GetYaw();  // Horizontal angle to tag (degrees)
    double pitch = bestTarget.GetPitch();  // Vertical angle to tag (degrees)
    
    // Rough distance estimate based on pitch and camera height
    // This is simplified - for production, use the full pose transform
    double estimatedDistance = CAMERA_HEIGHT_METERS / std::tan(degreesToRadians(pitch + CAMERA_PITCH_RADIANS));
    if (estimatedDistance < 0) estimatedDistance = 5.0;  // Sanity check
    
    // Calculate error from target distance
    double distanceError = estimatedDistance - m_targetDistance;
    
    // Simple P controller for forward speed
    double forwardSpeed = kP_Forward * distanceError;
    forwardSpeed = std::clamp(forwardSpeed, -kMaxSpeed, kMaxSpeed);
    
    // Simple P controller for rotation (align with tag)
    double rotationSpeed = -kP_Rotation * degreesToRadians(yaw);
    rotationSpeed = std::clamp(rotationSpeed, -1.0, 1.0);
    
    // Drive toward tag
    ChassisSpeed speeds;
    speeds.vx = forwardSpeed;
    speeds.vy = 0.0;
    speeds.omega = rotationSpeed;
    m_drivetrain->drive(speeds);
    
    // Update dashboard
    frc::SmartDashboard::PutNumber("Auto/Distance", estimatedDistance);
    frc::SmartDashboard::PutNumber("Auto/Yaw", yaw);
    frc::SmartDashboard::PutString("Auto Status", "Approaching tag");
    
    // Print progress occasionally
    static double lastPrint = 0.0;
    double now = frc::Timer::GetFPGATimestamp().value();
    if (now - lastPrint > 0.5) {
        printf("Distance: %.2fm, Yaw: %.1f°, Speed: %.2f m/s\n", 
               estimatedDistance, yaw, forwardSpeed);
        lastPrint = now;
    }
}

void DriveToAprilTagCommand::End(bool interrupted) {
    m_drivetrain->stop();
    
    if (interrupted) {
        printf("DriveToAprilTagCommand interrupted\n");
        frc::SmartDashboard::PutString("Auto Status", "Interrupted");
    } else {
        printf("DriveToAprilTagCommand completed - reached target\n");
        frc::SmartDashboard::PutString("Auto Status", "Target reached");
    }
}

bool DriveToAprilTagCommand::IsFinished() {
    // Timeout check
    double elapsed = frc::Timer::GetFPGATimestamp().value() - m_startTime;
    if (elapsed > m_timeoutSeconds) {
        printf("DriveToAprilTagCommand timed out after %.1fs\n", elapsed);
        return true;
    }
    
    // Check if we're at target distance
    if (!m_vision->hasTargets()) {
        return false;  // Keep running until we see a tag
    }
    
    auto result = m_vision->getLatestResult();
    if (!result.HasTargets()) {
        return false;
    }
    
    auto bestTarget = result.GetBestTarget();
    double pitch = bestTarget.GetPitch();
    double estimatedDistance = CAMERA_HEIGHT_METERS / std::tan(degreesToRadians(pitch + CAMERA_PITCH_RADIANS));
    
    // Check if within tolerance
    if (std::abs(estimatedDistance - m_targetDistance) < kDistanceTolerance) {
        return true;  // We're close enough!
    }
    
    return false;
}

