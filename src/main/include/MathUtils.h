#pragma once

#include <cmath>

/**
 * Clean mathematical utilities for swerve drive
 * Modern C++ implementation without the old "v2" naming
 */

struct Vector2D {
    double x = 0.0;
    double y = 0.0;
    
    Vector2D() = default;
    Vector2D(double x_val, double y_val) : x(x_val), y(y_val) {}
    
    // Vector operations
    Vector2D operator+(const Vector2D& other) const {
        return {x + other.x, y + other.y};
    }
    
    Vector2D operator-(const Vector2D& other) const {
        return {x - other.x, y - other.y};
    }
    
    Vector2D operator*(double scalar) const {
        return {x * scalar, y * scalar};
    }
    
    Vector2D operator/(double scalar) const {
        return {x / scalar, y / scalar};
    }
    
    // Utility functions
    double magnitude() const {
        return std::sqrt(x * x + y * y);
    }
    
    Vector2D normalize() const {
        double mag = magnitude();
        if (mag == 0.0) return {0.0, 0.0};
        return {x / mag, y / mag};
    }
    
    Vector2D rotate(double angle) const {
        double cos_a = std::cos(angle);
        double sin_a = std::sin(angle);
        return {
            x * cos_a - y * sin_a,
            x * sin_a + y * cos_a
        };
    }
    
    double dot(const Vector2D& other) const {
        return x * other.x + y * other.y;
    }
};

struct SwerveModuleState {
    double speed = 0.0;     // m/s
    double angle = 0.0;     // radians
};

struct ChassisSpeed {
    double vx = 0.0;        // m/s forward
    double vy = 0.0;        // m/s left  
    double omega = 0.0;     // rad/s counterclockwise
};

// Utility functions
inline double constrainAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

inline double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

inline double radiansToDegrees(double radians) {
    return radians * 180.0 / M_PI;
}

inline double applyDeadband(double value, double deadband) {
    return std::abs(value) > deadband ? value : 0.0;
}
