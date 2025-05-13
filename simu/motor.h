// motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <algorithm>
#include <cmath>
#include "Constants.h"

class Motor {
private:
    float _Ktau = 0.53f;      // Torque constant (Nm/A)
    float _Ke = 0.53f;        // Back-EMF constant ((V·s)/rad)
    float _R = 4.7177f;          // Resistance (Ohm)
    float _L = 0.00363f;      // Inductance (H)
    float _J = 0.0019f;       // Inertia (kg·m^2)
    float _b = 0.0595f;       // Viscous friction coefficient
    float _m_time = _J/_b;
    float _N_cur = 1/_m_time;
    float _N_vel = _N_cur/2;
    float _N_pos = _N_vel/2;

    float _fric_v_outer = 1.8f;
    float _fric_v_inner = 4.f;

    // State variables
    float _voltage = 0.0f;    // Control input voltage (V)
    float _current = 0.0f;    // Armature current (A)
    float _omega = 0.0f;      // Angular speed (rad/s)
    float _position = 0.0f;   // Angular position (rad)
    float _targetPosition = 0.0f; // Target position (rad)

    // PID parameters
    float _kp = 8.f;
    float _ki = 0.6f;
    float _kd = 4.0f;
    float _integral = 0.0f;
    float _previousError = 0.0f;

    // Optional maximum angular speed
    float _maxOmega = 2000.0f;  // Adjust based on your system

public:
    Motor() = default;

    void setVoltage(float voltage) {
        _voltage = voltage;
    }
    
    // Update motor state based on physics (call each frame)
    void update(float deltaTime);

    float compute(float deltaTime);
    
    // Set the target position for the motor
    void setTargetPosition(float position);
    
    // Getters
    float getCurrent() const { return _current; }
    float getOmega() const { return _omega; }
    float getPosition() const { return _position; }
};

#endif // MOTOR_H
