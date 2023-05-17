using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class Rotor
{
    private bool _isClockwise;
    private float _throttle = 0.0f; // between 0 and 1
    private float _desiredRotorSpeed = 0.0f; // [RPM]
    private float _rotorSpeed = 0.0f; // [RPM]
    private float _propellerDiameter; // [m]
    private float _rotorMomentOfInertia; // [kg*m^2]
    private float _rotorThrustCoefficient; // [N/RPM^2]
    private float _rotorTorqueCoefficient; // [Nm/RPM^2]
    private float _rotorThrust = 0.0f; // [N]
    private float _rotorTorque = 0.0f; // [N*m]
    private float _timeConstant; // [s]
    private float _rotorConstant; // [RPM] slope of linear relationship between throttle command and motor speed
    private float _omegaB; // [RPM] constant term of linear relationship between throttle command and motor speed

    public Rotor(bool isClockwise, float propellerDiameter, float rotorMomentOfInertia, float rotorThrustCoefficient, float rotorTorqueCoefficient, float timeConstant, float rotorConstant, float omegaB)
    {
        _isClockwise = isClockwise;
        _propellerDiameter = propellerDiameter;
        _rotorMomentOfInertia = rotorMomentOfInertia;
        _rotorThrustCoefficient = rotorThrustCoefficient;
        _rotorTorqueCoefficient = rotorTorqueCoefficient;
        _timeConstant = timeConstant;
        _rotorConstant = rotorConstant;
        _omegaB = omegaB;
    }

    public void Step(float dT)
    {
        _rotorSpeed = dT * ((_desiredRotorSpeed - _rotorSpeed) / _timeConstant) + _rotorSpeed;
        _rotorThrust = _rotorThrustCoefficient * _rotorSpeed * _rotorSpeed;
        _rotorTorque = _rotorTorqueCoefficient * _rotorSpeed * _rotorSpeed;
        if (_isClockwise)
        {
            _rotorTorque = -_rotorTorque; // clockwise rotors produce negative torque in body frame
        }
    }

    public void SetThrottle(float throttle)
    {
        _throttle = throttle;
        _desiredRotorSpeed = _rotorConstant * _throttle + _omegaB;
    }

    public float GetThrust()
    {
        return _rotorThrust;
    }

    public float GetTorque()
    {
        return _rotorTorque;
    }

    public Vector3 GetGyroscopicTorque(Vector3 angularVelocityB)
    {
        Vector3 gyroscopicTorque = new Vector3(0, 0, 0);
        if (_isClockwise)
        {
            gyroscopicTorque = new Vector3(-_rotorMomentOfInertia * angularVelocityB.y * (_rotorSpeed / 60f * 2f * Mathf.PI), _rotorMomentOfInertia * angularVelocityB.x * (_rotorSpeed / 60f * 2f * Mathf.PI), 0.0f);
        }
        else
        {
            gyroscopicTorque = new Vector3(_rotorMomentOfInertia * angularVelocityB.y * (_rotorSpeed / 60f * 2f * Mathf.PI), -_rotorMomentOfInertia * angularVelocityB.x * (_rotorSpeed / 60f * 2f * Mathf.PI), 0.0f);
        }
        return gyroscopicTorque;
    }
}
