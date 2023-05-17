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

    public PIDController(double dT, double setpoint, double kP, double kI, double kD)
    {
        _dT = dT;
        _setpoint = setpoint;
        _kP = kP;
        _kI = kI;
        _kD = kD;
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
        if (output > 1.0f)
        {
            output = 1.0f;
        }
        else if (output < 0.0f)
        {
            output = 0.0f;
        }
        _previousError = _error;

        return output;
    }

}
