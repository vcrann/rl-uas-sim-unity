using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class PIDController
{
    private double _dT;
    private double _kP;
    private double _kI;
    private double _kD;
    private double _setpoint;
    private double _error = 0.0;
    private double _previousError = 0.0;
    private double _integral = 0.0;
    private double _lowerLimit;
    private double _upperLimit;

    public PIDController(double dT, double setpoint, double kP, double kI, double kD, double lowerLimit, double upperLimit)
    {
        _dT = dT;
        _setpoint = setpoint;
        _kP = kP;
        _kI = kI;
        _kD = kD;
        _upperLimit = upperLimit;
        _lowerLimit = lowerLimit;
    }


    public double Calculate(double currentValue)
    {
        _error = _setpoint - currentValue;

        // Proportional
        double pOutput = _kP * _error;

        // Integral
        _integral += _error * _dT * _kI;
        double iOutput = _integral;

        // Derivative
        double dOutput = _kD * ((_error - _previousError) / _dT);

        double output = pOutput + iOutput + dOutput;
        if (output > _upperLimit)
        {
            output = 1.0f;
        }
        else if (output < _lowerLimit)
        {
            output = 0.0f;
        }
        _previousError = _error;

        return output;
    }

    public void SetSetpoint(double setpoint)
    {
        _setpoint = setpoint;
    }

    public double getSetpoint()
    {
        return _setpoint;
    }

}
