# ⚡ Zenitsu - FRC Swerve Robot

Clean, modern swerve drivetrain implementation for FRC 2026 season. Built with REV SparkMax motors, CTRE CANcoders, and NavX gyroscope.

## 🚀 Quick Start

### Hardware Requirements
- 4x REV SparkMax motor controllers (drive motors)
- 4x REV SparkMax motor controllers (steering motors) 
- 4x CTRE CANcoder absolute encoders
- 1x NavX gyroscope (MXP port)
- PlayStation DualShock controller

### Software Setup
1. **Install WPILib 2025** - [Download here](https://wpilib.org)
2. **Clone this repository**
   ```bash
   git clone <repo-url>
   cd zenitsu
   ```
3. **Build the project**
   ```bash
   ./gradlew build
   ```
4. **Deploy to robot**
   ```bash
   ./gradlew deploy
   ```

### First Time Setup
1. **Configure CAN IDs** in `src/main/include/Config.h` to match your robot
2. **Calibrate encoders** - See [Calibration Guide](docs/calibration.md)
3. **Test with controller** - See [Controller Guide](docs/controller.md)

## 🎮 Controller (PlayStation DualShock)
- **Left Stick**: Drive forward/backward, strafe left/right
- **Right Stick**: Rotate robot
- **L1**: Precision mode (slow, careful driving)
- **R1**: Turbo mode (fast driving)
- **L2**: Deadman switch (MUST hold to drive)
- **Share**: Toggle field-relative mode
- **Options**: Reset gyroscope
- **PS Button**: Emergency stop

## 📁 Project Structure
- `src/main/cpp/` - Source code
- `src/main/include/` - Header files  
- `src/test/cpp/` - Unit tests
- `docs/` - Documentation
- `Config.h` - Hardware configuration

## 🔧 Key Features
- ✅ Clean swerve kinematics
- ✅ PlayStation controller support
- ✅ NavX gyroscope integration
- ✅ Field-relative driving
- ✅ Safety systems (deadman switch, emergency stop)
- ✅ Encoder calibration system
- ✅ SmartDashboard telemetry

## 📚 Documentation
- [Controller Reference](docs/controller.md)
- [Calibration Procedures](docs/calibration.md) 
- [Development Guide](docs/development.md)

## 🚨 Safety
This robot requires **L2 (deadman switch) to be held** for all driving operations. Release L2 to immediately stop the robot.
