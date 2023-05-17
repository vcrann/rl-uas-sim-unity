using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class Rotor
{
    private bool _isClockwise;
    private double _throttle = 0.0; // between 0 and 1
    private double _desiredRotorSpeed = 0.0; // [RPM]
    private double _rotorSpeed = 0.0; // [RPM]
    private double _propellerDiameter; // [m]
    private double _rotorMomentOfInertia; // [kg*m^2]
    private double _rotorThrustCoefficient; // [N/RPM^2]
    private double _rotorTorqueCoefficient; // [Nm/RPM^2]
    private double _rotorThrust = 0.0; // [N]
    private double _rotorTorque = 0.0; // [N*m]
    private double _timeConstant; // [s]
    private double _rotorConstant; // [RPM] slope of linear relationship between throttle command and motor speed
    private double _omegaB; // [RPM] constant term of linear relationship between throttle command and motor speed

    public Rotor(bool isClockwise, double propellerDiameter, double rotorMomentOfInertia, double rotorThrustCoefficient, double rotorTorqueCoefficient, double timeConstant, double rotorConstant, double omegaB)
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

    public void Step(double dT)
    {
        _rotorSpeed = dT * ((_desiredRotorSpeed - _rotorSpeed) / _timeConstant) + _rotorSpeed;
        _rotorThrust = _rotorThrustCoefficient * _rotorSpeed * _rotorSpeed;
        _rotorTorque = _rotorTorqueCoefficient * _rotorSpeed * _rotorSpeed;
        if (_isClockwise)
        {
            _rotorTorque = -_rotorTorque; // clockwise rotors produce negative torque in body frame
        }
    }

    public void SetThrottle(double throttle)
    {
        _throttle = throttle;
        _desiredRotorSpeed = _rotorConstant * _throttle + _omegaB;
    }

    public double GetThrust()
    {
        return _rotorThrust;
    }

    public double GetTorque()
    {
        return _rotorTorque;
    }

    public Vector3d GetGyroscopicTorque(Vector3d angularVelocityB)
    {
        Vector3d gyroscopicTorque = new Vector3d(0, 0, 0);
        if (_isClockwise)
        {
            gyroscopicTorque = new Vector3d(-_rotorMomentOfInertia * angularVelocityB.y * (_rotorSpeed / 60 * 2 * Mathd.PI), _rotorMomentOfInertia * angularVelocityB.x * (_rotorSpeed / 60 * 2 * Mathd.PI), 0.0);
        }
        else
        {
            gyroscopicTorque = new Vector3d(_rotorMomentOfInertia * angularVelocityB.y * (_rotorSpeed / 60 * 2 * Mathd.PI), -_rotorMomentOfInertia * angularVelocityB.x * (_rotorSpeed / 60 * 2 * Mathd.PI), 0.0);
        }
        return gyroscopicTorque;
    }
}
