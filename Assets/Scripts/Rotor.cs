using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Rotor : MonoBehaviour
{
    private bool _isClockwise;
    private float _throttle;
    private float _desiredRotorSpeed;
    private float _rotorSpeed;
    private float _propellerDiameter;
    private float _rotorMomentOfInertia;
    private float _rotorThrustCoefficient;
    private float _rotorTorqueCoefficient;
    private float _rotorThrust;
    private float _rotorTorque;
    private float _timeConstant;
    private float _rotorConstant;
    private float _omegaB;
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

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
