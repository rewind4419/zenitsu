# 🚀 Deployment Guide

## Prerequisites
- **WPILib 2025** installed
- **Robot connected** via USB or WiFi
- **Team number configured** in WPILib
- **Vendordeps installed**: REVLib, Phoenix6, Studica (NavX)

## Quick Deploy
```bash
./gradlew deploy
```

## First Time Setup
1. **Configure CAN IDs** in `src/main/include/Config.h` to match your robot
2. **Build and deploy**:
   ```bash
   ./gradlew build
   ./gradlew deploy
   ```
3. **Calibrate encoders** - see [Calibration Guide](calibration.md)
4. (Laptop dev) Install toolchain once:
   ```bash
   ./gradlew installRoboRioToolchain
   ```

## Build Options
```bash
./gradlew build          # Build only
./gradlew deploy         # Build and deploy 
./gradlew testExternalNativeDebug  # Run native tests
```

## Troubleshooting Deploy
- **Can't connect to robot?** Check USB cable or WiFi
- **Wrong team number?** Update in VS Code: `Ctrl+Shift+P` → "Set Team Number"  
- **Build errors?** Check `Config.h` for syntax errors
- **Deploy hangs?** Try USB connection instead of WiFi

## Testing Checklist
After successful deploy:
- [ ] Connect PlayStation controller
- [ ] Enable robot in teleop
- [ ] Verify wheels respond to joystick
- [ ] Check SmartDashboard: `NavX Connected`, `Steer kP`, and the `Field` widget shows pose
- [ ] PS Button triggers emergency stop
