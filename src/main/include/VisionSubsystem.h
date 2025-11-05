#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <optional>
#include <memory>

#include "Config.h"

/**
 * Vision subsystem for AprilTag detection and pose estimation
 * Uses PhotonVision running on Orange Pi coprocessor
 * 
 * This subsystem is designed to be modular - when upgrading to Limelight 4,
 * only the implementation (.cpp) needs to change, not the interface.
 */
class VisionSubsystem : public frc2::SubsystemBase {
public:
    VisionSubsystem();
    
    /**
     * Get estimated robot pose from vision
     * @return Optional pose - has value if AprilTag(s) detected, empty otherwise
     */
    std::optional<photon::EstimatedRobotPose> getEstimatedGlobalPose();
    
    /**
     * Check if any AprilTags are currently visible
     * @return true if camera sees at least one AprilTag
     */
    bool hasTargets() const;
    
    /**
     * Get the latest camera pipeline result
     * @return Raw PhotonVision pipeline result for detailed analysis
     */
    photon::PhotonPipelineResult getLatestResult() const;
    
    /**
     * Get the ID of the best (closest/most confident) AprilTag
     * @return Tag ID, or -1 if no targets visible
     */
    int getBestTargetID() const;
    
    /**
     * Get the number of AprilTags currently visible
     * @return Count of detected tags
     */
    int getTargetCount() const;
    
    /**
     * Periodic update - called automatically by CommandScheduler
     */
    void Periodic() override;

private:
    // PhotonVision camera
    std::unique_ptr<photon::PhotonCamera> m_camera;
    
    // Pose estimator for converting AprilTag detections to robot pose
    std::unique_ptr<photon::PhotonPoseEstimator> m_poseEstimator;
    
    // Transform from robot center to camera
    frc::Transform3d m_robotToCamera;
    
    // AprilTag field layout (loaded from WPILib or custom)
    frc::AprilTagFieldLayout m_fieldLayout;
    
    /**
     * Create the robot-to-camera transform from Config.h values
     */
    frc::Transform3d createRobotToCameraTransform();
    
    /**
     * Load the AprilTag field layout
     * For 2025 season, this will load the official field layout
     * For testing, can create custom layout with test tags
     */
    frc::AprilTagFieldLayout loadFieldLayout();
};

