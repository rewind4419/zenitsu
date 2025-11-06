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
    
    // Initialize cached state
    m_hasValidResult = false;
    m_cameraConnected = false;
    
    printf("VisionSubsystem initialized with camera: %s\n", PHOTON_CAMERA_NAME);
    printf("NOTE: Camera will fail silently if PhotonVision is not connected\n");
}

std::optional<photon::EstimatedRobotPose> VisionSubsystem::getEstimatedGlobalPose() {
    // Use cached result to avoid multiple network calls
    if (!m_hasValidResult || !m_latestResult.HasTargets()) {
        return std::nullopt;
    }
    
    // Update the pose estimator with the cached result
    return m_poseEstimator->Update(m_latestResult);
}

bool VisionSubsystem::hasTargets() const {
    return m_hasValidResult && m_latestResult.HasTargets();
}

photon::PhotonPipelineResult VisionSubsystem::getLatestResult() const {
    return m_latestResult;
}

int VisionSubsystem::getBestTargetID() const {
    if (!m_hasValidResult || !m_latestResult.HasTargets()) {
        return -1;
    }
    
    // Get the best target (lowest ambiguity / closest)
    auto bestTarget = m_latestResult.GetBestTarget();
    return bestTarget.GetFiducialId();
}

int VisionSubsystem::getTargetCount() const {
    if (!m_hasValidResult || !m_latestResult.HasTargets()) {
        return 0;
    }
    return m_latestResult.GetTargets().size();
}

void VisionSubsystem::Periodic() {
    // Fetch results ONCE per loop cycle (20ms)
    // This is the ONLY place we call GetAllUnreadResults() to avoid network spam
    auto results = m_camera->GetAllUnreadResults();
    
    // Check if camera is connected
    if (results.empty()) {
        m_hasValidResult = false;
        m_cameraConnected = false;
        m_latestResult = photon::PhotonPipelineResult{};
        
        // PROMINENT WARNING on SmartDashboard
        frc::SmartDashboard::PutString("⚠️ VISION STATUS", "❌ CAMERA NOT CONNECTED");
        frc::SmartDashboard::PutBoolean("Vision/CameraConnected", false);
        frc::SmartDashboard::PutBoolean("Vision/HasTargets", false);
        frc::SmartDashboard::PutNumber("Vision/TargetCount", 0);
        frc::SmartDashboard::PutNumber("Vision/BestTargetID", -1);
        
        // Print warning once when connection is lost
        static bool lastConnectedState = true;
        if (lastConnectedState) {
            printf("⚠️ WARNING: PhotonVision camera '%s' not connected!\n", PHOTON_CAMERA_NAME);
            printf("   Check: 1) Orange Pi powered on, 2) Network connection, 3) PhotonVision running\n");
            lastConnectedState = false;
        }
        return;
    }
    
    // Camera is connected - cache the latest result
    m_hasValidResult = true;
    m_cameraConnected = true;
    m_latestResult = results.back();
    
    // Print message once when connection is established
    static bool lastConnectedState = false;
    if (!lastConnectedState) {
        printf("✓ PhotonVision camera '%s' connected!\n", PHOTON_CAMERA_NAME);
        lastConnectedState = true;
    }
    
    // Update SmartDashboard with vision status
    frc::SmartDashboard::PutString("⚠️ VISION STATUS", "✓ CAMERA CONNECTED");
    frc::SmartDashboard::PutBoolean("Vision/CameraConnected", true);
    frc::SmartDashboard::PutBoolean("Vision/HasTargets", m_latestResult.HasTargets());
    
    if (m_latestResult.HasTargets()) {
        frc::SmartDashboard::PutNumber("Vision/TargetCount", m_latestResult.GetTargets().size());
        frc::SmartDashboard::PutNumber("Vision/BestTargetID", getBestTargetID());
        
        auto bestTarget = m_latestResult.GetBestTarget();
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
    // TEMPORARY: Using 2025 Reefscape field for pre-season testing
    // We have a physical Reefscape field at school with AprilTags already mounted
    // This enables full pose estimation and autonomous navigation testing
    // 
    // TODO: After 2026 kickoff (January 2026), change to:
    // return frc::LoadAprilTagLayoutField(frc::AprilTagField::k2026<GameName>);
    
    printf("Loading 2025 Reefscape field layout for testing\n");
    printf("NOTE: This is temporary - update after 2026 kickoff!\n");
    
    auto layout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2025ReefscapeWelded);
    return layout;
}

