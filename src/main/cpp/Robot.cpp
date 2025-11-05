#include "Robot.h"
#include "Config.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/struct/Pose2dStruct.h>
#include <frc2/command/CommandScheduler.h>

Robot::Robot() {
    // Constructor - hardware will be initialized in RobotInit()
}

void Robot::RobotInit() {
    // Initialize DataLogManager for AdvantageScope analysis
    frc::DataLogManager::Start();
    printf("DataLogManager started for AdvantageScope logging\n");
    
    // Initialize hardware
    m_drivetrain = std::make_unique<Drivetrain>();
    m_gamepadInput = std::make_unique<GamepadInput>(DRIVER_CONTROLLER_PORT);
    m_vision = std::make_unique<VisionSubsystem>();
    
    // Initialize NavX via Studica library (if available in this environment)
    #if NAVX_AVAILABLE
    try {
        m_navx = std::make_unique<studica::AHRS>(studica::AHRS::NavXComType::kMXP_SPI);
        printf("NavX gyroscope initialized via Studica library\n");
    } catch (std::exception& ex) {
        printf("NavX initialization failed: %s\n", ex.what());
    }
    #else
    printf("NavX headers not available in this build environment (CI sim). Skipping NavX init.\n");
    #endif
    
    // Initialize dashboard
    frc::SmartDashboard::PutBoolean("Field Relative", m_fieldRelative);
    frc::SmartDashboard::PutString("Robot State", "Initialized");

    // Show field view for odometry (CONFIGURABLE: use during autos/tuning)
    frc::SmartDashboard::PutData("Field", &m_field);
    
    // Create teleop drive command
    #if NAVX_AVAILABLE
    void* navxPtr = static_cast<void*>(m_navx.get());
    #else
    void* navxPtr = nullptr;
    #endif
    m_teleopDriveCommand = std::make_unique<TeleopDriveCommand>(
        m_drivetrain.get(),
        m_gamepadInput.get(),
        navxPtr,
        &m_fieldRelative
    );
    
    printf("Zenitsu Robot Initialized! Ready for lightning-fast swerve driving with PlayStation controller!\n");
    printf("NavX gyroscope connected for field-relative driving\n");
}

void Robot::RobotPeriodic() {
    // Run the CommandScheduler - this handles all subsystem periodic methods and command execution
    frc2::CommandScheduler::GetInstance().Run();
    
    // Update input every loop
    m_gamepadInput->update();
    
    // Update telemetry
    updateDashboard();
    
    // Log drivetrain performance for AdvantageScope
    logDrivetrainPerformance();

    // Update odometry and field visualization when NavX is present
    #if NAVX_AVAILABLE
    auto pose = m_drivetrain->updateOdometry(m_navx->IsConnected() ? degreesToRadians(m_navx->GetYaw() + NAVX_YAW_OFFSET_DEGREES) : 0.0);
    m_field.SetRobotPose(pose);
    #endif
    
    // Update pose estimation with vision measurements (Kalman filter fusion)
    updateVisionPoseEstimation();
}

void Robot::AutonomousInit() {
    frc::SmartDashboard::PutString("Robot State", "Autonomous");
    
    // Reset auto timer
    m_autoStartTime = frc::Timer::GetFPGATimestamp().value();
    printf("Starting autonomous: drive forwards at 50%% for 5 seconds\n");
}

void Robot::AutonomousPeriodic() {
    double elapsed = frc::Timer::GetFPGATimestamp().value() - m_autoStartTime;
    
    if (elapsed < 5.0) {
        // Drive forwards at 50% speed for 5 seconds
        ChassisSpeed speeds;
        speeds.vx = 0.5 * MAX_DRIVE_SPEED;   // Forwards (positive)
        speeds.vy = 0.0;                      // No strafe
        speeds.omega = 0.0;                   // No rotation
        m_drivetrain->drive(speeds);
        
        // Print status every 0.5 seconds
        static double lastPrint = 0.0;
        if (elapsed - lastPrint > 0.5) {
            printf("Auto: %.1fs - driving forwards\n", elapsed);
            lastPrint = elapsed;
        }
    } else {
        // Stop after 5 seconds
        m_drivetrain->stop();
    }
}

void Robot::TeleopInit() {
    frc::SmartDashboard::PutString("Robot State", "Teleop");
    
    // Schedule the teleop drive command as the default command
    m_drivetrain->SetDefaultCommand(std::move(*m_teleopDriveCommand));
    
    // Print calibration data on teleop start
    printf("\n========== CALIBRATION DATA ==========\n");
    auto raw = m_drivetrain->getRawModuleAngles();
    printf("Copy these values to Config.h if recalibrating:\n");
    printf("constexpr double FRONT_LEFT_ENCODER_OFFSET  = %.4f;\n", raw[0]);
    printf("constexpr double FRONT_RIGHT_ENCODER_OFFSET = %.4f;\n", raw[1]);
    printf("constexpr double BACK_LEFT_ENCODER_OFFSET   = %.4f;\n", raw[2]);
    printf("constexpr double BACK_RIGHT_ENCODER_OFFSET  = %.4f;\n", raw[3]);
    printf("======================================\n\n");
}

void Robot::TeleopPeriodic() {
    // CommandScheduler handles command execution in RobotPeriodic()
    // No manual calls needed here
}

void Robot::DisabledInit() {
    frc::SmartDashboard::PutString("Robot State", "Disabled");
    
    // Stop all motors when disabled
    m_drivetrain->stop();
}

void Robot::DisabledPeriodic() {
    // Keep motors stopped while disabled
    m_drivetrain->stop();
}

void Robot::updateDashboard() {
    // Drivetrain telemetry is now handled by Drivetrain::Periodic()
    
    // Update NavX telemetry
    #if NAVX_AVAILABLE
    if (m_navx->IsConnected()) {
        frc::SmartDashboard::PutNumber("Gyro Yaw", m_navx->GetYaw());
        frc::SmartDashboard::PutNumber("Gyro Pitch", m_navx->GetPitch());
        frc::SmartDashboard::PutNumber("Gyro Roll", m_navx->GetRoll());
        frc::SmartDashboard::PutBoolean("NavX Connected", true);
    } else {
        frc::SmartDashboard::PutBoolean("NavX Connected", false);
    }
    #else
    frc::SmartDashboard::PutString("Gyro Status", "NavX headers not present in CI build");
    #endif
    
    // Update input info
    Vector2D translation = m_gamepadInput->getDriveTranslation();
    frc::SmartDashboard::PutNumber("Drive X", translation.x);
    frc::SmartDashboard::PutNumber("Drive Y", translation.y);
    frc::SmartDashboard::PutNumber("Rotation", m_gamepadInput->getRotation());
    
    // Update safety status
    frc::SmartDashboard::PutBoolean("Controller Connected", m_gamepadInput->isControllerConnected());
    
    // Update control mode info
    frc::SmartDashboard::PutBoolean("Turbo Mode (R1)", m_gamepadInput->isTurboMode());
    frc::SmartDashboard::PutNumber("Speed Multiplier", m_gamepadInput->getSpeedMultiplier());
    
    // Vision diagnostic mode (Share + L1)
    bool shareBtn = m_gamepadInput->getShareButton();
    bool l1Btn = m_gamepadInput->isPrecisionMode();
    if (shareBtn && l1Btn) {
        // Vision diagnostic mode - print detailed vision info (throttled to 1 Hz)
        frc::SmartDashboard::PutString("Diag Mode", "Vision Test");
        
        static double lastPrintTime = 0.0;
        double currentTime = frc::Timer::GetFPGATimestamp().value();
        
        // Only print once per second
        if (currentTime - lastPrintTime >= 1.0) {
            lastPrintTime = currentTime;
            
            bool hasTargets = m_vision->hasTargets();
            int tagCount = m_vision->getTargetCount();
            int bestTagID = m_vision->getBestTargetID();
            
            printf("=== VISION DIAGNOSTIC ===\n");
            printf("Has Targets: %s\n", hasTargets ? "YES" : "NO");
            printf("Tag Count: %d\n", tagCount);
            printf("Best Tag ID: %d\n", bestTagID);
            
            if (hasTargets) {
                auto visionPoseEstimate = m_vision->getEstimatedGlobalPose();
                if (visionPoseEstimate.has_value()) {
                    auto estimatedPose = visionPoseEstimate.value();
                    frc::Pose2d visionPose = estimatedPose.estimatedPose.ToPose2d();
                    
                    printf("Vision Pose: X=%.2fm, Y=%.2fm, Rot=%.1f°\n",
                           visionPose.X().value(),
                           visionPose.Y().value(),
                           visionPose.Rotation().Degrees().value());
                    printf("Timestamp: %.3fs\n", estimatedPose.timestamp.value());
                    printf("Confidence: %s\n", tagCount >= 2 ? "HIGH (multi-tag)" : "MEDIUM (single-tag)");
                } else {
                    printf("Vision pose estimation failed\n");
                }
            }
            
            // Compare with fused pose
            auto fusedPose = m_drivetrain->getPose();
            printf("Fused Pose: X=%.2fm, Y=%.2fm, Rot=%.1f°\n",
                   fusedPose.X().value(),
                   fusedPose.Y().value(),
                   fusedPose.Rotation().Degrees().value());
            printf("========================\n");
        }
    }
}

void Robot::logDrivetrainPerformance() {
    // Get DataLog reference
    wpi::log::DataLog& log = frc::DataLogManager::GetLog();
    
    // Create static log entries (created once, reused every loop)
    // Robot pose for 3D field visualization
    static wpi::log::StructLogEntry<frc::Pose2d> logPose(log, "Drivetrain/Pose");
    
    // Chassis speeds
    static wpi::log::DoubleLogEntry logChassisVx(log, "Drivetrain/Chassis/vx");
    static wpi::log::DoubleLogEntry logChassisVy(log, "Drivetrain/Chassis/vy");
    static wpi::log::DoubleLogEntry logChassisOmega(log, "Drivetrain/Chassis/omega");
    
    // Front Left module
    static wpi::log::DoubleLogEntry logFLDesiredSpeed(log, "Drivetrain/FL/Desired/Speed");
    static wpi::log::DoubleLogEntry logFLActualSpeed(log, "Drivetrain/FL/Actual/Speed");
    static wpi::log::DoubleLogEntry logFLDesiredAngle(log, "Drivetrain/FL/Desired/Angle");
    static wpi::log::DoubleLogEntry logFLActualAngle(log, "Drivetrain/FL/Actual/Angle");
    static wpi::log::DoubleLogEntry logFLDriveOutput(log, "Drivetrain/FL/Drive/AppliedOutput");
    static wpi::log::DoubleLogEntry logFLSteerOutput(log, "Drivetrain/FL/Steer/AppliedOutput");
    static wpi::log::DoubleLogEntry logFLDriveCurrent(log, "Drivetrain/FL/Drive/Current");
    static wpi::log::DoubleLogEntry logFLSteerCurrent(log, "Drivetrain/FL/Steer/Current");
    
    // Front Right module
    static wpi::log::DoubleLogEntry logFRDesiredSpeed(log, "Drivetrain/FR/Desired/Speed");
    static wpi::log::DoubleLogEntry logFRActualSpeed(log, "Drivetrain/FR/Actual/Speed");
    static wpi::log::DoubleLogEntry logFRDesiredAngle(log, "Drivetrain/FR/Desired/Angle");
    static wpi::log::DoubleLogEntry logFRActualAngle(log, "Drivetrain/FR/Actual/Angle");
    static wpi::log::DoubleLogEntry logFRDriveOutput(log, "Drivetrain/FR/Drive/AppliedOutput");
    static wpi::log::DoubleLogEntry logFRSteerOutput(log, "Drivetrain/FR/Steer/AppliedOutput");
    static wpi::log::DoubleLogEntry logFRDriveCurrent(log, "Drivetrain/FR/Drive/Current");
    static wpi::log::DoubleLogEntry logFRSteerCurrent(log, "Drivetrain/FR/Steer/Current");
    
    // Back Left module
    static wpi::log::DoubleLogEntry logBLDesiredSpeed(log, "Drivetrain/BL/Desired/Speed");
    static wpi::log::DoubleLogEntry logBLActualSpeed(log, "Drivetrain/BL/Actual/Speed");
    static wpi::log::DoubleLogEntry logBLDesiredAngle(log, "Drivetrain/BL/Desired/Angle");
    static wpi::log::DoubleLogEntry logBLActualAngle(log, "Drivetrain/BL/Actual/Angle");
    static wpi::log::DoubleLogEntry logBLDriveOutput(log, "Drivetrain/BL/Drive/AppliedOutput");
    static wpi::log::DoubleLogEntry logBLSteerOutput(log, "Drivetrain/BL/Steer/AppliedOutput");
    static wpi::log::DoubleLogEntry logBLDriveCurrent(log, "Drivetrain/BL/Drive/Current");
    static wpi::log::DoubleLogEntry logBLSteerCurrent(log, "Drivetrain/BL/Steer/Current");
    
    // Back Right module
    static wpi::log::DoubleLogEntry logBRDesiredSpeed(log, "Drivetrain/BR/Desired/Speed");
    static wpi::log::DoubleLogEntry logBRActualSpeed(log, "Drivetrain/BR/Actual/Speed");
    static wpi::log::DoubleLogEntry logBRDesiredAngle(log, "Drivetrain/BR/Desired/Angle");
    static wpi::log::DoubleLogEntry logBRActualAngle(log, "Drivetrain/BR/Actual/Angle");
    static wpi::log::DoubleLogEntry logBRDriveOutput(log, "Drivetrain/BR/Drive/AppliedOutput");
    static wpi::log::DoubleLogEntry logBRSteerOutput(log, "Drivetrain/BR/Steer/AppliedOutput");
    static wpi::log::DoubleLogEntry logBRDriveCurrent(log, "Drivetrain/BR/Drive/Current");
    static wpi::log::DoubleLogEntry logBRSteerCurrent(log, "Drivetrain/BR/Steer/Current");
    
    // Vision system telemetry
    static wpi::log::BooleanLogEntry logVisionHasTargets(log, "Vision/HasTargets");
    static wpi::log::IntegerLogEntry logVisionTargetCount(log, "Vision/TargetCount");
    static wpi::log::IntegerLogEntry logVisionBestTagID(log, "Vision/BestTagID");
    static wpi::log::DoubleLogEntry logVisionPoseX(log, "Vision/EstimatedPose/X");
    static wpi::log::DoubleLogEntry logVisionPoseY(log, "Vision/EstimatedPose/Y");
    static wpi::log::DoubleLogEntry logVisionPoseRotation(log, "Vision/EstimatedPose/Rotation");
    static wpi::log::DoubleLogEntry logVisionTimestamp(log, "Vision/Timestamp");
    static wpi::log::DoubleLogEntry logVisionStdDevX(log, "Vision/StdDev/X");
    static wpi::log::DoubleLogEntry logVisionStdDevY(log, "Vision/StdDev/Y");
    static wpi::log::DoubleLogEntry logVisionStdDevTheta(log, "Vision/StdDev/Theta");
    
    // Get data from drivetrain
    auto commandedSpeeds = m_drivetrain->getLastCommandedSpeeds();
    auto actualStates = m_drivetrain->getModuleStates();
    const auto& modules = m_drivetrain->getModules();
    
    // Log chassis speeds
    logChassisVx.Append(commandedSpeeds.vx);
    logChassisVy.Append(commandedSpeeds.vy);
    logChassisOmega.Append(commandedSpeeds.omega);
    
    // Log Front Left (index 0)
    logFLDesiredSpeed.Append(modules[0]->getDesiredSpeed());
    logFLActualSpeed.Append(actualStates[0].speed);
    logFLDesiredAngle.Append(radiansToDegrees(modules[0]->getDesiredAngle()));
    logFLActualAngle.Append(radiansToDegrees(actualStates[0].angle));
    logFLDriveOutput.Append(modules[0]->getDriveAppliedOutput());
    logFLSteerOutput.Append(modules[0]->getSteerAppliedOutput());
    logFLDriveCurrent.Append(modules[0]->getDriveOutputCurrent());
    logFLSteerCurrent.Append(modules[0]->getSteerOutputCurrent());
    
    // Log Front Right (index 1)
    logFRDesiredSpeed.Append(modules[1]->getDesiredSpeed());
    logFRActualSpeed.Append(actualStates[1].speed);
    logFRDesiredAngle.Append(radiansToDegrees(modules[1]->getDesiredAngle()));
    logFRActualAngle.Append(radiansToDegrees(actualStates[1].angle));
    logFRDriveOutput.Append(modules[1]->getDriveAppliedOutput());
    logFRSteerOutput.Append(modules[1]->getSteerAppliedOutput());
    logFRDriveCurrent.Append(modules[1]->getDriveOutputCurrent());
    logFRSteerCurrent.Append(modules[1]->getSteerOutputCurrent());
    
    // Log Back Left (index 2)
    logBLDesiredSpeed.Append(modules[2]->getDesiredSpeed());
    logBLActualSpeed.Append(actualStates[2].speed);
    logBLDesiredAngle.Append(radiansToDegrees(modules[2]->getDesiredAngle()));
    logBLActualAngle.Append(radiansToDegrees(actualStates[2].angle));
    logBLDriveOutput.Append(modules[2]->getDriveAppliedOutput());
    logBLSteerOutput.Append(modules[2]->getSteerAppliedOutput());
    logBLDriveCurrent.Append(modules[2]->getDriveOutputCurrent());
    logBLSteerCurrent.Append(modules[2]->getSteerOutputCurrent());
    
    // Log Back Right (index 3)
    logBRDesiredSpeed.Append(modules[3]->getDesiredSpeed());
    logBRActualSpeed.Append(actualStates[3].speed);
    logBRDesiredAngle.Append(radiansToDegrees(modules[3]->getDesiredAngle()));
    logBRActualAngle.Append(radiansToDegrees(actualStates[3].angle));
    logBRDriveOutput.Append(modules[3]->getDriveAppliedOutput());
    logBRSteerOutput.Append(modules[3]->getSteerAppliedOutput());
    logBRDriveCurrent.Append(modules[3]->getDriveOutputCurrent());
    logBRSteerCurrent.Append(modules[3]->getSteerOutputCurrent());
    
    // Log robot pose for 3D field visualization in AdvantageScope
    logPose.Append(m_drivetrain->getPose());
    
    // Log vision system data
    bool hasTargets = m_vision->hasTargets();
    logVisionHasTargets.Append(hasTargets);
    logVisionTargetCount.Append(m_vision->getTargetCount());
    logVisionBestTagID.Append(m_vision->getBestTargetID());
    
    // Log vision pose estimate if available
    auto visionPoseEstimate = m_vision->getEstimatedGlobalPose();
    if (visionPoseEstimate.has_value()) {
        auto estimatedPose = visionPoseEstimate.value();
        frc::Pose2d visionPose = estimatedPose.estimatedPose.ToPose2d();
        
        logVisionPoseX.Append(visionPose.X().value());
        logVisionPoseY.Append(visionPose.Y().value());
        logVisionPoseRotation.Append(visionPose.Rotation().Degrees().value());
        logVisionTimestamp.Append(estimatedPose.timestamp.value());
        
        // Log the standard deviations used for this measurement
        int tagCount = m_vision->getTargetCount();
        if (tagCount >= 2) {
            logVisionStdDevX.Append(VISION_STD_DEV_X_MULTI);
            logVisionStdDevY.Append(VISION_STD_DEV_Y_MULTI);
            logVisionStdDevTheta.Append(VISION_STD_DEV_THETA_MULTI);
        } else {
            logVisionStdDevX.Append(VISION_STD_DEV_X_SINGLE);
            logVisionStdDevY.Append(VISION_STD_DEV_Y_SINGLE);
            logVisionStdDevTheta.Append(VISION_STD_DEV_THETA_SINGLE);
        }
    } else {
        // No vision data - log zeros or invalid values
        logVisionPoseX.Append(0.0);
        logVisionPoseY.Append(0.0);
        logVisionPoseRotation.Append(0.0);
        logVisionTimestamp.Append(0.0);
        logVisionStdDevX.Append(0.0);
        logVisionStdDevY.Append(0.0);
        logVisionStdDevTheta.Append(0.0);
    }
}

void Robot::updateVisionPoseEstimation() {
    // Get estimated pose from vision system
    auto visionPoseEstimate = m_vision->getEstimatedGlobalPose();
    
    // If no vision measurement available, return early
    if (!visionPoseEstimate.has_value()) {
        return;
    }
    
    // Extract pose and timestamp
    auto estimatedPose = visionPoseEstimate.value();
    frc::Pose2d visionPose = estimatedPose.estimatedPose.ToPose2d();
    double timestamp = estimatedPose.timestamp.value();
    
    // Determine standard deviations based on number of tags detected
    // More tags = more confident measurement = lower std devs
    int tagCount = m_vision->getTargetCount();
    wpi::array<double, 3> stdDevs = (tagCount >= 2) 
        ? wpi::array<double, 3>{VISION_STD_DEV_X_MULTI, VISION_STD_DEV_Y_MULTI, VISION_STD_DEV_THETA_MULTI}
        : wpi::array<double, 3>{VISION_STD_DEV_X_SINGLE, VISION_STD_DEV_Y_SINGLE, VISION_STD_DEV_THETA_SINGLE};
    
    // Add vision measurement to pose estimator (Kalman filter fusion)
    m_drivetrain->addVisionMeasurement(visionPose, timestamp, stdDevs);
}
