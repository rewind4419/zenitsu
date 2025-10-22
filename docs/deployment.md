# 🚀 Deployment Guide

## Prerequisites
- **WPILib 2025** installed
- **Robot connected** via USB or WiFi
- **Team number configured** in WPILib

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

## Build Options
```bash
./gradlew build          # Build only
./gradlew deploy         # Build and deploy 
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
- [ ] Test deadman switch (L2)
- [ ] Verify wheels respond to joystick
- [ ] Check SmartDashboard for sensor data
