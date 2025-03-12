// motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <algorithm>
#include <cmath>
#include "Constants.h"

class Motor {
private:
    float _targetPosition = 0.0f;     // Target angle in degrees
    float _currentPosition = 0.0f;    // Current angle in degrees
    float _currentSpeed = 0.0f;       // Current rotational speed (deg/sec)
    float _previousSpeed = 0.0f;      // Previous speed for derivative calculation
    float _previousError = 0.0f;
    float _filteredDerivative = 0.0f;

    // PD controller parameters
    const float _kp = 1.4f;           // Proportional gain
    const float _kd = 0.3f;           // Derivative gain

    // Motor specifications
    const float _maxSpeed = 1536.0f;    // Max speed in degrees/sec
    const float _maxAccel = 1500.0f;    // Max acceleration in degrees/sec²
    const float _friction = 0.25f;     // Motor friction coefficient

public:
    Motor() = default;
    
    // Update motor state based on physics (call each frame)
    void update(float deltaTime);
    
    // Set the target position for the motor
    void setTargetPosition(float position);
    
    // Get current position
    float getCurrentPosition() const { return _currentPosition; }
    
    // Get current speed
    float getCurrentSpeed() const { return _currentSpeed; }
};

#endif // MOTOR_H