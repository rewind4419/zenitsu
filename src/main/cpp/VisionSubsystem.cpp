#include "VisionSubsystem.h"
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/length.h>

VisionSubsystem::VisionSubsystem() {
    // Create PhotonCamera instance
    m_camera = std::make_unique<photon::PhotonCamera>(PHOTON_CAMERA_NAME);
    
    // Create robot-to-camera transform
    m_robotToCamera = createRobotToCameraTransform();
    
    // Load AprilTag field layout
    m_fieldLayout = loadFieldLayout();
    
    // Create pose estimator
    m_poseEstimator = std::make_unique<photon::PhotonPoseEstimator>(
        m_fieldLayout,
        photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        m_robotToCamera
    );
    
    // Set fallback strategy for single tag detection
    m_poseEstimator->SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);
    
    printf("VisionSubsystem initialized with camera: %s\n", PHOTON_CAMERA_NAME);
}

std::optional<photon::EstimatedRobotPose> VisionSubsystem::getEstimatedGlobalPose() {
    // Get all unread results from the camera
    auto results = m_camera->GetAllUnreadResults();
    
    // If no results, return empty
    if (results.empty()) {
        return std::nullopt;
    }
    
    // Use the most recent result (last in the vector)
    auto result = results.back();
    
    // If no targets detected, return empty
    if (!result.HasTargets()) {
        return std::nullopt;
    }
    
    // Update the pose estimator with the latest result
    return m_poseEstimator->Update(result);
}

bool VisionSubsystem::hasTargets() const {
    auto results = m_camera->GetAllUnreadResults();
    if (results.empty()) {
        return false;
    }
    return results.back().HasTargets();
}

photon::PhotonPipelineResult VisionSubsystem::getLatestResult() const {
    auto results = m_camera->GetAllUnreadResults();
    if (results.empty()) {
        return photon::PhotonPipelineResult{};  // Return empty result
    }
    return results.back();
}

int VisionSubsystem::getBestTargetID() const {
    auto results = m_camera->GetAllUnreadResults();
    if (results.empty()) {
        return -1;
    }
    
    auto result = results.back();
    if (!result.HasTargets()) {
        return -1;
    }
    
    // Get the best target (lowest ambiguity / closest)
    auto bestTarget = result.GetBestTarget();
    return bestTarget.GetFiducialId();
}

int VisionSubsystem::getTargetCount() const {
    auto results = m_camera->GetAllUnreadResults();
    if (results.empty()) {
        return 0;
    }
    
    auto result = results.back();
    if (!result.HasTargets()) {
        return 0;
    }
    return result.GetTargets().size();
}

void VisionSubsystem::Periodic() {
    // Update SmartDashboard with vision status
    auto results = m_camera->GetAllUnreadResults();
    if (results.empty()) {
        frc::SmartDashboard::PutBoolean("Vision/HasTargets", false);
        frc::SmartDashboard::PutNumber("Vision/TargetCount", 0);
        frc::SmartDashboard::PutNumber("Vision/BestTargetID", -1);
        return;
    }
    
    auto result = results.back();
    frc::SmartDashboard::PutBoolean("Vision/HasTargets", result.HasTargets());
    
    if (result.HasTargets()) {
        frc::SmartDashboard::PutNumber("Vision/TargetCount", result.GetTargets().size());
        frc::SmartDashboard::PutNumber("Vision/BestTargetID", getBestTargetID());
        
        auto bestTarget = result.GetBestTarget();
        frc::SmartDashboard::PutNumber("Vision/TargetYaw", bestTarget.GetYaw());
        frc::SmartDashboard::PutNumber("Vision/TargetPitch", bestTarget.GetPitch());
        frc::SmartDashboard::PutNumber("Vision/TargetArea", bestTarget.GetArea());
    } else {
        frc::SmartDashboard::PutNumber("Vision/TargetCount", 0);
        frc::SmartDashboard::PutNumber("Vision/BestTargetID", -1);
    }
}

frc::Transform3d VisionSubsystem::createRobotToCameraTransform() {
    // Create translation from robot center to camera
    frc::Translation3d translation{
        units::meter_t{CAMERA_X_OFFSET_METERS},
        units::meter_t{CAMERA_Y_OFFSET_METERS},
        units::meter_t{CAMERA_HEIGHT_METERS}
    };
    
    // Create rotation for camera pitch and yaw
    frc::Rotation3d rotation{
        units::radian_t{0.0},                    // Roll (camera shouldn't roll)
        units::radian_t{CAMERA_PITCH_RADIANS},   // Pitch (up/down tilt)
        units::radian_t{CAMERA_YAW_RADIANS}      // Yaw (left/right rotation)
    };
    
    return frc::Transform3d{translation, rotation};
}

frc::AprilTagFieldLayout VisionSubsystem::loadFieldLayout() {
    // For 2026 season - field layout will be available after kickoff in January 2026
    // Until then, create an empty layout for testing with custom AprilTags
    
    printf("Creating empty AprilTag field layout for 2026 season\n");
    printf("Update this after kickoff (January 2026) to load official field layout\n");
    
    // Create an empty layout for testing
    // Teams can add custom test tags here for pre-season testing
    frc::AprilTagFieldLayout emptyLayout{
        {},  // Empty tag list - add test tags as needed
        units::meter_t{16.54},  // Field length (standard FRC field)
        units::meter_t{8.21}    // Field width (standard FRC field)
    };
    
    return emptyLayout;
    
    // TODO: After 2026 kickoff, replace the above with:
    // return frc::LoadAprilTagLayoutField(frc::AprilTagField::k2026<GameName>);
}

