# 🔧 Calibration Guide

## Encoder Calibration (First Time Setup)

**Why?** Swerve wheels need to know which direction is "forward"

### Steps
1. **Enter calibration mode**: Hold `L1 + R1 + Options` 
2. **Disable robot** and manually align all wheels pointing forward
3. **Enable robot** - check SmartDashboard for encoder values
4. **Record the values** for each wheel
5. **Update Config.h** with the recorded offset values:
   ```cpp
   constexpr double FRONT_LEFT_ENCODER_OFFSET = YOUR_VALUE;
   constexpr double FRONT_RIGHT_ENCODER_OFFSET = YOUR_VALUE; 
   // etc...
   ```
6. **Deploy** and test - wheels should align properly now

## Gyro Setup
1. **Mount NavX** level on robot chassis
2. **Connect to MXP port** on roboRIO  
3. **Reset gyro** with Options button when robot is straight

## Quick Test
- All wheels point forward when stopped ✓
- Robot drives straight with forward stick ✓  
- Field-relative mode works ✓
