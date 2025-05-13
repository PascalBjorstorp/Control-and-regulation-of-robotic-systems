// motor.cpp
#include "motor.h"

void Motor::update(float deltaTime) {
    if (deltaTime <= 0) return; // Avoid division by zero

    _current = (_voltage - _Ke * _omega) / _R;

    float dOmega = (_Ktau * _current - _b * _omega) / _J;

    _omega += dOmega * deltaTime;
    _omega = std::clamp(_omega, -_maxOmega, _maxOmega);
    _position += _omega * deltaTime;
}

float Motor::compute(float deltaTime){
    if(deltaTime <= 0) return 0.0f;

    float error = _targetPosition - _position;
    _integral += error * deltaTime;
    float derivative = (error - _previousError) / deltaTime;

    float controlSignal = _kp * error + _ki * _integral + _kd * derivative;
    controlSignal = std::clamp(controlSignal, -6.0f, 6.0f);

    _previousError = error;
    _voltage = controlSignal;

    return controlSignal;
}

void Motor::setTargetPosition(float position) {
    _targetPosition = position;
}
