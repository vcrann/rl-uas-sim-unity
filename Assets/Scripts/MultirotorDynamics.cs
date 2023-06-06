using System.Collections;
using System.Collections.Generic;
using System.Timers;
using UnityEngine;
public class MultirotorDynamics
{
    // State Variables
    private Vector3d _positionE = new Vector3d(0, 0, 0);
    private Vector3d _velocityB = new Vector3d(0, 0, 0);
    private Vector3d _accelerationB = new Vector3d(0, 0, 0);
    private Vector3d _attitudeE = new Vector3d(0, 0, 0);
    private Vector3d _angularVelocityB = new Vector3d(0, 0, 0);
    private Vector3d _angularAccelerationB = new Vector3d(0, 0, 0);
    private Vector3d _thrustB = new Vector3d(0, 0, 0);
    private Vector3d _torqueB = new Vector3d(0, 0, 0);

    // Aircraft Physical Parameters TODO load from file
    private double _mass = 1.0f;
    private double _inertiaXx = 4.856e-3f;
    private double _inertiaYy = 4.856e-3f;
    private double _inertiaZz = 8.801e-3f;
    private int _numberOfRotors = 4;
    Rotor[] _rotors = new Rotor[4];
    private double[] _armLengths = new double[4] { 0.1, 0.1, 0.1, 0.1 };
    private double[] _armAngles = new double[4] { Mathd.PI / 4.0, 3.0 * Mathd.PI / 4.0, 5.0 * Mathd.PI / 4.0, 7.0 * Mathd.PI / 4.0 };
    private double _linearDragCoefficient = 0.425;
    private double _referenceArea = 0.015;

    // Environment Parameters
    private double _airDensity = 1.225;
    private Vector3d _gravityE = new Vector3d(0, 0, 9.81);
    private double _dT = 0.001;

    //Controllers
    private PIDController _altitudeController = new PIDController(0.001, -1, -0.1, -0.05, -0.2, 0.0, 1.0);
    private PIDController _rollRateController = new PIDController(0.001, 0, 0.5, 0.05, 0.1, -1.0, 1.0);
    private PIDController _pitchRateController = new PIDController(0.001, 0, 0.5, 0.1, 0.1, -1.0, 1.0);
    private PIDController _yawRateController = new PIDController(0.001, 0, 0.5, 0.05, 0.1, -1.0, 1.0);
    private PIDController _rollAngleController = new PIDController(0.001, 0, 5.0, 0.0, 0.0, -1.0, 1.0);
    private PIDController _pitchAngleController = new PIDController(0.001, 0, 5.0, 0.0, 0.0, -1.0, 1.0);
    private PIDController _yawAngleController = new PIDController(0.001, 0, 0.5, 0.0, 0.0, -1.0, 1.0);

    private PIDController _velocityControllerX = new PIDController(0.001, 0, 1, 0.1, 0.1, -10.0, 10.0);
    private PIDController _velocityControllerY = new PIDController(0.001, 0, 1, 0.1, 0.1, -10.0, 10.0);
    private PIDController _velocityControllerZ = new PIDController(0.001, 0, 1, 0.1, 0.1, -10.0, 10.0);
    private PIDController _positionControllerX = new PIDController(0.001, 0, 1, 0.0, 0.0, -2.0, 2.0);
    private PIDController _positionControllerY = new PIDController(0.001, 0, 1, 0.0, 0.0, -2.0, 2.0);
    private PIDController _positionControllerZ = new PIDController(0.001, 0, 1, 0.0, 0.0, -2.0, 2.0);

    // Lock for thread safety
    private readonly object _stateLock = new object();

    public void RunModel()
    {
        // Temp code to test the model
        _rotors[0] = new Rotor(true, 0.05, 3.357e-5, 0.0000001984, 0.000000003733, 0.098, 6432, 1779);
        _rotors[1] = new Rotor(false, 0.05, 3.357e-5, 0.0000001984, 0.000000003733, 0.098, 6432, 1779);
        _rotors[2] = new Rotor(true, 0.05, 3.357e-5, 0.0000001984, 0.000000003733, 0.098, 6432, 1779);
        _rotors[3] = new Rotor(false, 0.05, 3.357e-5, 0.0000001984, 0.000000003733, 0.098, 6432, 1779);

        // Create a timer with a 1ms interval (1000Hz)
        Timer modelTimer = new System.Timers.Timer(1);
        // Hook up the Elapsed event for the timer
        modelTimer.Elapsed += OnTimedEvent;
        modelTimer.AutoReset = true;
        modelTimer.Enabled = true;
    }

    private void OnTimedEvent(object source, System.Timers.ElapsedEventArgs e)
    {

        lock (_stateLock)
        {
            Step(_dT);
        }
        PositionToVelocity();
        _rollAngleController.SetSetpoint(_velocityControllerY.Calculate(_velocityB.y));
        _pitchAngleController.SetSetpoint(-_velocityControllerX.Calculate(_velocityB.x));
        //Debug.Log(_velocityControllerY.Calculate(_velocityB.y) + " " + -_velocityControllerX.Calculate(_velocityB.x));
        AngleToRate();
        QuadMixer(_altitudeController.Calculate(_positionE.z), _rollRateController.Calculate(_angularVelocityB.x), _pitchRateController.Calculate(_angularVelocityB.y), _yawRateController.Calculate(_angularVelocityB.z));

    }



    void Step(double dT)
    {
        // Calculate the thrust and torque
        _thrustB = new Vector3d(0, 0, 0);
        _torqueB = new Vector3d(0, 0, 0);
        for (int i = 0; i <= _numberOfRotors - 1; i++)
        {
            _rotors[i].Step(dT);
            _thrustB.z -= _rotors[i].GetThrust(); // thrust upwards is negative in body frame, can only ever be upwards
            _torqueB += new Vector3d(-_armLengths[i] * _rotors[i].GetThrust() * Mathd.Sin(_armAngles[i]), _armLengths[i] * _rotors[i].GetThrust() * Mathd.Cos(_armAngles[i]), _rotors[i].GetTorque());
            _torqueB += _rotors[i].GetGyroscopicTorque(_angularVelocityB);
        }

        // Linear
        _accelerationB = (_thrustB / _mass) + (CalculateLinearDrag() / _mass) + EarthToBody(_gravityE, _attitudeE);
        _velocityB += _accelerationB * dT;
        _positionE += BodyToEarth(_velocityB, _attitudeE) * dT;

        //Angular
        Vector3d _inertia = new Vector3d(_inertiaXx, _inertiaYy, _inertiaZz);
        Vector3d _unscaledAngularAccelerationB = (-Vector3d.Cross(_angularVelocityB, Vector3d.Scale(_inertia, _angularVelocityB)) + _torqueB);
        _angularAccelerationB.x = _unscaledAngularAccelerationB.x / _inertia.x;
        _angularAccelerationB.y = _unscaledAngularAccelerationB.y / _inertia.y;
        _angularAccelerationB.z = _unscaledAngularAccelerationB.z / _inertia.z;

        _angularVelocityB += _angularAccelerationB * dT;
        _attitudeE += _angularVelocityB * dT;
    }

    void AngleToRate()
    {
        _rollRateController.SetSetpoint(_rollAngleController.Calculate(_attitudeE.x));
        _pitchRateController.SetSetpoint(_pitchAngleController.Calculate(_attitudeE.y));
        //_yawRateController.SetSetpoint(_yawAngleController.Calculate(_attitudeE.z));
        //Debug.Log(_rollAngleController.Calculate(_attitudeE.x) + " " + _pitchAngleController.Calculate(_attitudeE.y));
    }

    void PositionToVelocity()
    {
        Vector3d positionControllerOutput = new Vector3d(_positionControllerX.Calculate(_positionE.x), _positionControllerY.Calculate(_positionE.y), _positionE.z);
        positionControllerOutput = EarthToBody(positionControllerOutput, _attitudeE);
        _velocityControllerX.SetSetpoint(positionControllerOutput.x);
        _velocityControllerY.SetSetpoint(positionControllerOutput.y);
        //Debug.Log("Position Controller Output: " + positionControllerOutput);
    }

    void QuadMixer(double altitudeThrottleModifier, double rollModifier, double pitchModifier, double yawModifier) // only works for Quad X but its a start
    {
        _rotors[0].SetThrottle(altitudeThrottleModifier - rollModifier * altitudeThrottleModifier + pitchModifier * altitudeThrottleModifier - yawModifier * altitudeThrottleModifier);
        _rotors[1].SetThrottle(altitudeThrottleModifier - rollModifier * altitudeThrottleModifier - pitchModifier * altitudeThrottleModifier + yawModifier * altitudeThrottleModifier);
        _rotors[2].SetThrottle(altitudeThrottleModifier + rollModifier * altitudeThrottleModifier - pitchModifier * altitudeThrottleModifier - yawModifier * altitudeThrottleModifier);
        _rotors[3].SetThrottle(altitudeThrottleModifier + rollModifier * altitudeThrottleModifier + pitchModifier * altitudeThrottleModifier + yawModifier * altitudeThrottleModifier);
    }

    Vector3d BodyToEarth(Vector3d vectorB, Vector3d rotationB)
    {
        double x = vectorB.x;
        double y = vectorB.y;
        double z = vectorB.z;
        double phi = rotationB.x;
        double theta = rotationB.y;
        double psi = rotationB.z;

        double[,] rotationMatrixBToE = new double[3, 3];
        rotationMatrixBToE[0, 0] = Mathd.Cos(psi) * Mathd.Cos(theta);
        rotationMatrixBToE[0, 1] = Mathd.Cos(psi) * Mathd.Sin(theta) * Mathd.Sin(phi) - Mathd.Sin(psi) * Mathd.Cos(phi);
        rotationMatrixBToE[0, 2] = Mathd.Cos(psi) * Mathd.Sin(theta) * Mathd.Cos(phi) + Mathd.Sin(psi) * Mathd.Sin(phi);
        rotationMatrixBToE[1, 0] = Mathd.Sin(psi) * Mathd.Cos(theta);
        rotationMatrixBToE[1, 1] = Mathd.Sin(psi) * Mathd.Sin(theta) * Mathd.Sin(phi) + Mathd.Cos(psi) * Mathd.Cos(phi);
        rotationMatrixBToE[1, 2] = Mathd.Sin(psi) * Mathd.Sin(theta) * Mathd.Cos(phi) - Mathd.Cos(psi) * Mathd.Sin(phi);
        rotationMatrixBToE[2, 0] = -Mathd.Sin(theta);
        rotationMatrixBToE[2, 1] = Mathd.Cos(theta) * Mathd.Sin(phi);
        rotationMatrixBToE[2, 2] = Mathd.Cos(theta) * Mathd.Cos(phi);

        Vector3d result = new Vector3d(
            rotationMatrixBToE[0, 0] * x + rotationMatrixBToE[0, 1] * y + rotationMatrixBToE[0, 2] * z,
            rotationMatrixBToE[1, 0] * x + rotationMatrixBToE[1, 1] * y + rotationMatrixBToE[1, 2] * z,
            rotationMatrixBToE[2, 0] * x + rotationMatrixBToE[2, 1] * y + rotationMatrixBToE[2, 2] * z
        );
        return result;
    }

    // method to convert coordinates from earth frame to body frame
    Vector3d EarthToBody(Vector3d vectorE, Vector3d rotationB)
    {
        double x = vectorE.x;
        double y = vectorE.y;
        double z = vectorE.z;
        double phi = rotationB.x;
        double theta = rotationB.y;
        double psi = rotationB.z;

        double[,] rotationMatrixEToB = new double[3, 3];
        rotationMatrixEToB[0, 0] = Mathd.Cos(psi) * Mathd.Cos(theta);
        rotationMatrixEToB[0, 1] = Mathd.Sin(psi) * Mathd.Cos(theta);
        rotationMatrixEToB[0, 2] = -Mathd.Sin(theta);
        rotationMatrixEToB[1, 0] = Mathd.Cos(psi) * Mathd.Sin(theta) * Mathd.Sin(phi) - Mathd.Sin(psi) * Mathd.Cos(phi);
        rotationMatrixEToB[1, 1] = Mathd.Sin(psi) * Mathd.Sin(theta) * Mathd.Sin(phi) + Mathd.Cos(psi) * Mathd.Cos(phi);
        rotationMatrixEToB[1, 2] = Mathd.Cos(theta) * Mathd.Sin(phi);
        rotationMatrixEToB[2, 0] = Mathd.Cos(psi) * Mathd.Sin(theta) * Mathd.Cos(phi) + Mathd.Sin(psi) * Mathd.Sin(phi);
        rotationMatrixEToB[2, 1] = Mathd.Sin(psi) * Mathd.Sin(theta) * Mathd.Cos(phi) - Mathd.Cos(psi) * Mathd.Sin(phi);
        rotationMatrixEToB[2, 2] = Mathd.Cos(theta) * Mathd.Cos(phi);

        Vector3d result = new Vector3d(
            rotationMatrixEToB[0, 0] * x + rotationMatrixEToB[0, 1] * y + rotationMatrixEToB[0, 2] * z,
            rotationMatrixEToB[1, 0] * x + rotationMatrixEToB[1, 1] * y + rotationMatrixEToB[1, 2] * z,
            rotationMatrixEToB[2, 0] * x + rotationMatrixEToB[2, 1] * y + rotationMatrixEToB[2, 2] * z
        );
        return result;
    }

    public Vector3d GetPosition()
    {
        return _positionE;
    }

    public Vector3d GetAttitude()
    {
        return _attitudeE;
    }

    //TODO implement angular drag and improve linear with blade flapping etc
    Vector3d CalculateLinearDrag()
    {
        Vector3d drag = new Vector3d();
        //double effectiveArea = _topArea * Mathd.Sin(_attitudeE.x) + _sideArea * Mathd.Sin(_attitudeE.y);
        drag = -(0.5 * _airDensity * _velocityB.magnitude * _velocityB.magnitude * _linearDragCoefficient * _referenceArea) * _velocityB.normalized;
        return drag;
    }

    // Vector3d CalculateAngularDrag()
    // {
    //     // Vector3d drag = new Vector3d();
    //     // drag.x = -_angularDragCoefficient * _angularVelocityB.x * _angularVelocityB.x * _angularVelocityB.x / Mathd.Abs(_angularVelocityB.x);
    //     // drag.y = -_angularDragCoefficient * _angularVelocityB.y * _angularVelocityB.y * _angularVelocityB.y / Mathd.Abs(_angularVelocityB.y);
    //     // drag.z = -_angularDragCoefficient * _angularVelocityB.z * _angularVelocityB.z * _angularVelocityB.z / Mathd.Abs(_angularVelocityB.z);
    //     // return drag;
    // }

    public double[] GetRotorSpeeds()
    {
        double[] _rotorSpeeds = new double[4];
        for (int i = 0; i < 4; i++)
        {
            _rotorSpeeds[i] = _rotors[i].GetRotorSpeed();
        }
        return _rotorSpeeds;
    }

    public void GetManualInput(float rollInput, float pitchInput, float yawInput, float altitudeInput)
    {
        _rollAngleController.SetSetpoint(rollInput * 0.174533);
        _pitchAngleController.SetSetpoint(-pitchInput * 0.174533);
        _yawRateController.SetSetpoint(yawInput * 2);
        _altitudeController.SetSetpoint(_altitudeController.GetSetpoint() - altitudeInput * 0.005f);
    }

    public void SetDesiredPosition(Vector3 desiredPosition)
    {
        _positionControllerX.SetSetpoint(desiredPosition.x);
        _positionControllerY.SetSetpoint(desiredPosition.y);
    }
}