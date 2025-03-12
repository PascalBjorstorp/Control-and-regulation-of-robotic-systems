// motor.cpp
#include "motor.h"

void Motor::update(float deltaTime) {
    if (deltaTime <= 0) return; // Avoid division by zero

    // Calculate error
    float error = _targetPosition - _currentPosition;

    // PD controller with filtered derivative
    float P = _kp * error;
    float rawDerivative = (error - _previousError) / deltaTime;
    
    // Apply a simple low-pass filter to smooth the derivative
    float alpha = 0.8f;  // Smoothing factor (adjustable)
    _filteredDerivative = alpha * _filteredDerivative + (1.0f - alpha) * rawDerivative;
    
    float D = _kd * _filteredDerivative;
    float controlSignal = P + D;

    // Apply physical limits of the motor
    controlSignal = std::clamp(controlSignal, -_maxAccel, _maxAccel);

    // Update speed based on control signal
    _currentSpeed += controlSignal * deltaTime;

    // Improved friction model (static + dynamic)
    float dynamicFriction = _friction * std::abs(_currentSpeed);
    float staticFriction = (_friction * 2.0f) * (1.0f / (std::abs(_currentSpeed) + 1.0f));
    float frictionForce = dynamicFriction + staticFriction;
    _currentSpeed *= (1.0f - frictionForce * deltaTime);

    // Physical speed limit of the motor
    _currentSpeed = std::clamp(_currentSpeed, -_maxSpeed, _maxSpeed);

    // Update position
    _currentPosition += _currentSpeed * deltaTime;

    // Store current error for next update
    _previousError = error;
}

void Motor::setTargetPosition(float position) {
    _targetPosition = position;
}