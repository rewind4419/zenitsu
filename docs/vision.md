# Vision System Documentation

## Overview

The Zenitsu robot uses vision-based localization with AprilTags for autonomous navigation. The system fuses camera detections with wheel odometry and gyroscope data using a Kalman filter for robust pose estimation.

**Current Setup:**
- **Camera:** PhotonVision on Orange Pi coprocessor
- **Future Upgrade:** Limelight 4 (easy swap - same interface)
- **Field:** 2025 Reefscape (temporary for pre-season testing)
- **Season:** Preparing for 2026 (will update field layout after kickoff)

---

## Hardware Setup

### 1. Orange Pi Configuration

**Install PhotonVision:**
1. Download the latest PhotonVision release for Orange Pi from: https://github.com/PhotonVision/photonvision/releases
2. Flash the image to an SD card using balenaEtcher or similar
3. Insert SD card into Orange Pi and power on
4. Connect Orange Pi to robot network (Ethernet recommended for reliability)

**Access PhotonVision Dashboard:**
- Connect to robot WiFi
- Navigate to: `http://photonvision.local:5800` (or use robot IP)
- Default credentials: no login required

### 2. Camera Calibration

**CRITICAL: Calibrate your camera before use!**

1. In PhotonVision dashboard, go to "Cameras" tab
2. Select your camera (should auto-detect)
3. Go to "Calibration" section
4. Print a calibration board (chessboard pattern) from PhotonVision docs
5. Follow on-screen instructions to capture calibration images
6. Run calibration - aim for <0.5 pixel reprojection error
7. Save calibration data

**Why calibration matters:**
- Uncalibrated cameras produce inaccurate pose estimates
- Can cause robot to drive in wrong direction or miss targets
- Takes 5-10 minutes but saves hours of debugging

### 3. Camera Mounting

**Physical Installation:**
- Mount camera securely to robot frame (no vibration!)
- Position for clear view of field AprilTags
- Avoid obstructions from robot mechanisms
- Protect lens from impacts

**Measure Camera Position:**
You MUST measure these values accurately and update `Config.h`:

```cpp
// In Config.h - VISION CONFIGURATION section
constexpr double CAMERA_HEIGHT_METERS = 0.5;     // Height of lens above floor
constexpr double CAMERA_PITCH_RADIANS = 0.0;     // Tilt angle (0 = horizontal, + = up)
constexpr double CAMERA_YAW_RADIANS = 0.0;       // Rotation (0 = forward)
constexpr double CAMERA_X_OFFSET_METERS = 0.0;   // Forward/back from robot center
constexpr double CAMERA_Y_OFFSET_METERS = 0.0;   // Left/right from robot center
```

**How to measure:**
- `CAMERA_HEIGHT_METERS`: Tape measure from floor to lens center
- `CAMERA_PITCH_RADIANS`: Use protractor or phone level app (convert degrees to radians: deg * π/180)
- `CAMERA_YAW_RADIANS`: Angle relative to robot forward direction
- `CAMERA_X_OFFSET_METERS`: Measure from robot center to camera (+ = forward)
- `CAMERA_Y_OFFSET_METERS`: Measure from robot center to camera (+ = left)

### 4. PhotonVision Pipeline Configuration

**Create AprilTag Pipeline:**
1. In PhotonVision dashboard, go to "Pipelines" tab
2. Click "Add Pipeline" → "AprilTag"
3. Name it something descriptive (e.g., "Competition_AprilTags")
4. Configure settings:
   - **Tag Family:** 36h11 (FRC standard)
   - **Decimation:** 2.0 (balance speed vs accuracy)
   - **Blur:** 0.0 (unless camera is noisy)
   - **Threads:** 4 (use all cores)
   - **Pose Estimation:** Multi-target (best accuracy)
5. Save pipeline

**Set Camera Name:**
- In "Cameras" tab, rename camera to match `Config.h`:
  ```cpp
  constexpr const char* PHOTON_CAMERA_NAME = "OrangePi_Camera";
  ```
- This name MUST match exactly (case-sensitive)

---

## Software Configuration

### 1. Camera Parameters (Config.h)

Already configured in `Config.h` - verify these match your physical setup:

```cpp
// VISION CONFIGURATION (PhotonVision)
constexpr const char* PHOTON_CAMERA_NAME = "OrangePi_Camera";

// Camera mounting position (MEASURE THESE!)
constexpr double CAMERA_HEIGHT_METERS = 0.5;
constexpr double CAMERA_PITCH_RADIANS = 0.0;
constexpr double CAMERA_YAW_RADIANS = 0.0;
constexpr double CAMERA_X_OFFSET_METERS = 0.0;
constexpr double CAMERA_Y_OFFSET_METERS = 0.0;
```

### 2. Kalman Filter Tuning (Config.h)

The pose estimator uses standard deviations to determine trust levels:

```cpp
// Single tag detection (less confident)
constexpr double VISION_STD_DEV_X_SINGLE = 0.5;      // meters
constexpr double VISION_STD_DEV_Y_SINGLE = 0.5;      // meters
constexpr double VISION_STD_DEV_THETA_SINGLE = 0.8;  // radians

// Multiple tag detection (more confident)
constexpr double VISION_STD_DEV_X_MULTI = 0.1;       // meters
constexpr double VISION_STD_DEV_Y_MULTI = 0.1;       // meters
constexpr double VISION_STD_DEV_THETA_MULTI = 0.2;   // radians
```

**What these mean:**
- **Lower values** = trust vision more (pose estimate changes quickly)
- **Higher values** = trust odometry more (pose estimate changes slowly)
- **Single-tag** values are higher (less confident) because one tag can be ambiguous
- **Multi-tag** values are lower (more confident) because multiple tags cross-validate

**When to tune:**
- Robot "jumps" when seeing tags → increase vision std devs (trust less)
- Robot ignores vision corrections → decrease vision std devs (trust more)
- Start with defaults, then adjust based on testing

### 3. Field Layout

**Current Setup (Pre-Season 2026):**
```cpp
// In VisionSubsystem.cpp - loadFieldLayout()
auto layout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2025ReefscapeWelded);
```

We're using the 2025 Reefscape field because:
- We have a physical Reefscape setup at school
- Enables full testing of vision and autonomous navigation
- 2026 game hasn't been released yet

**After 2026 Kickoff:**
Update `VisionSubsystem.cpp`:
```cpp
// Change to 2026 field layout (replace <GameName> with actual game)
auto layout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2026<GameName>);
```

---

## Testing & Diagnostics

### 1. Vision Diagnostic Mode (Share + L1)

**How to use:**
1. Deploy code to robot
2. Enable robot (teleop or disabled mode)
3. Hold **Share + L1** buttons on controller
4. Watch console output (prints once per second)

**What you'll see:**
```
=== VISION DIAGNOSTIC ===
Has Targets: YES
Tag Count: 2
Best Tag ID: 5
Vision Pose: X=3.45m, Y=2.10m, Rot=45.0°
Timestamp: 123.456s
Confidence: HIGH (multi-tag)
Fused Pose: X=3.42m, Y=2.08m, Rot=44.8°
========================
```

**What to check:**
- **Has Targets:** Should be "YES" when AprilTags are visible
- **Tag Count:** Higher is better (2+ = high confidence)
- **Best Tag ID:** Should match visible tag numbers on field
- **Vision Pose:** Should roughly match robot's actual position
- **Fused Pose:** Should be close to vision pose (Kalman filter smooths it)

### 2. AdvantageScope Analysis

**View Vision Data:**
1. Download log files from robot (`/home/lvuser/logs/`)
2. Open in AdvantageScope
3. Navigate to vision data:
   - `Vision/HasTargets` - boolean, true when tags visible
   - `Vision/TargetCount` - number of tags detected
   - `Vision/BestTagID` - ID of primary tag
   - `Vision/EstimatedPose/X` - vision X position (meters)
   - `Vision/EstimatedPose/Y` - vision Y position (meters)
   - `Vision/EstimatedPose/Rotation` - vision heading (degrees)
   - `Vision/Timestamp` - measurement timestamp
   - `Vision/StdDev/*` - Kalman filter trust levels

**3D Field View:**
1. In AdvantageScope, select "3D Field" tab
2. Add robot pose: `Odometry/Pose` (fused odometry + vision)
3. Add vision pose: `Vision/EstimatedPose` (raw vision)
4. Load 2025 Reefscape field layout
5. Play back log to see robot movement

**What to look for:**
- Vision pose should "snap" to correct position when tags appear
- Fused pose should smoothly blend odometry and vision
- No wild jumps or oscillations (if present, tune std devs)

### 3. SmartDashboard Monitoring

**Real-time values:**
- `Vision/HasTargets` - Live tag detection status
- `Vision/TargetCount` - Number of visible tags
- `Vision/BestTargetID` - Primary tag ID
- `Field` - 2D field view with robot pose (blue arrow)

---

## Autonomous Commands

### DriveToAprilTag Command

**What it does:**
- Drives robot toward nearest visible AprilTag
- Stops at configurable distance (default 1.5m)
- Uses simple proportional control

**How to use:**
1. Place robot on field facing AprilTags
2. Enable autonomous mode
3. Robot will automatically drive toward tag
4. Stops when within 1.5m or after 10-second timeout

**Configuration:**
```cpp
// In Robot.cpp - RobotInit()
m_autoCommand = std::make_unique<DriveToAprilTagCommand>(
    m_drivetrain.get(),
    m_vision.get(),
    1.5,  // Target distance (meters)
    10.0  // Timeout (seconds)
);
```

**Tuning:**
Edit `DriveToAprilTagCommand.h` constants:
```cpp
static constexpr double kP_Forward = 0.5;   // Forward speed gain (higher = faster approach)
static constexpr double kP_Rotation = 2.0;  // Rotation gain (higher = faster turning)
static constexpr double kMaxSpeed = 0.5;    // Max speed (m/s)
```

**Limitations:**
- Basic P controller (no acceleration limits)
- No obstacle avoidance
- Single-tag targeting only
- For production, consider PathPlanner or similar

---

## Troubleshooting

### Problem: "No targets detected" in diagnostic mode

**Possible causes:**
1. **Camera not connected**
   - Check Orange Pi power and network connection
   - Verify PhotonVision dashboard is accessible
   - Check camera name matches `Config.h`

2. **AprilTags not visible**
   - Ensure tags are in camera field of view
   - Check lighting (too bright/dark affects detection)
   - Verify tags are printed correctly (36h11 family)

3. **Pipeline not configured**
   - Open PhotonVision dashboard
   - Verify AprilTag pipeline is active
   - Check tag family is set to 36h11

### Problem: Vision pose is wildly incorrect

**Possible causes:**
1. **Camera not calibrated**
   - MUST calibrate camera (see Hardware Setup)
   - Recalibrate if camera was bumped/moved

2. **Wrong camera mounting parameters**
   - Re-measure camera position/angle
   - Update `Config.h` with correct values
   - Even small errors (5cm, 5°) cause big problems

3. **Wrong field layout**
   - Verify field layout matches physical field
   - Check AprilTag positions are correct

### Problem: Robot "jumps" when seeing tags

**Cause:** Vision std devs too low (trusting vision too much)

**Fix:** Increase vision standard deviations in `Config.h`:
```cpp
// Make these values LARGER to trust vision less
constexpr double VISION_STD_DEV_X_SINGLE = 1.0;  // was 0.5
constexpr double VISION_STD_DEV_Y_SINGLE = 1.0;  // was 0.5
constexpr double VISION_STD_DEV_THETA_SINGLE = 1.5;  // was 0.8
```

### Problem: Robot ignores vision corrections

**Cause:** Vision std devs too high (trusting odometry too much)

**Fix:** Decrease vision standard deviations in `Config.h`:
```cpp
// Make these values SMALLER to trust vision more
constexpr double VISION_STD_DEV_X_MULTI = 0.05;  // was 0.1
constexpr double VISION_STD_DEV_Y_MULTI = 0.05;  // was 0.1
constexpr double VISION_STD_DEV_THETA_MULTI = 0.1;  // was 0.2
```

### Problem: DriveToAprilTag doesn't move

**Possible causes:**
1. **No tags visible** - Check diagnostic mode (Share + L1)
2. **Already at target distance** - Robot stops at 1.5m by default
3. **Command not scheduled** - Check console for "DriveToAprilTag command scheduled"

---

## Upgrading to Limelight 4

When you're ready to upgrade from Orange Pi to Limelight 4:

**Hardware:**
1. Mount Limelight 4 in same position as Orange Pi camera
2. Connect to robot network (Ethernet)
3. Configure Limelight pipeline (AprilTag mode)

**Software (minimal changes):**
1. Add Limelight vendor dependency to `vendordeps/`
2. Update `VisionSubsystem.cpp`:
   - Replace `PhotonCamera` with `LimelightHelpers`
   - Update `getEstimatedGlobalPose()` to use Limelight API
   - Keep same interface (no changes to Robot.cpp needed!)

**Why this is easy:**
- `VisionSubsystem` interface stays the same
- Only implementation changes, not the API
- All logging, diagnostics, and commands work unchanged

---

## Best Practices

### 1. Always Calibrate Camera
- Calibrate before first use
- Re-calibrate if camera is bumped or moved
- Check calibration periodically (once per season minimum)

### 2. Measure Camera Position Accurately
- Use tape measure and level
- Double-check measurements
- Update `Config.h` immediately after mounting

### 3. Test Vision Before Competition
- Use diagnostic mode (Share + L1) to verify detection
- Drive around field and check pose accuracy
- Review logs in AdvantageScope

### 4. Start Conservative, Then Tune
- Use default std dev values initially
- Only tune if you see problems (jumps, ignoring vision)
- Make small changes (10-20% at a time)

### 5. Monitor Vision Health
- Check `Vision/HasTargets` on SmartDashboard
- Watch for consistent tag detection
- Alert drivers if vision is lost

---

## Additional Resources

**PhotonVision Documentation:**
- https://docs.photonvision.org/

**WPILib Pose Estimation:**
- https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html

**AprilTag Information:**
- https://april.eecs.umich.edu/software/apriltag

**FRC Field Layouts:**
- https://github.com/wpilibsuite/allwpilib/tree/main/apriltag/src/main/native/resources

---

## Support

**Questions or issues?**
- Check this documentation first
- Review PhotonVision docs
- Ask on Chief Delphi (FRC community forum)
- Consult with programming mentors

**Found a bug?**
- Document the issue (screenshots, logs)
- Note what you were doing when it happened
- Share with team programming lead

