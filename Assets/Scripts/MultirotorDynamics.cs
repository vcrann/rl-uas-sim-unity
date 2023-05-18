using System.Collections;
using System.Collections.Generic;
using System.Timers;
using UnityEngine;
public class MultirotorDynamics
{
    // Start is called before the first frame update
    private Vector3d _positionE = new Vector3d(0, 0, 0);
    private Vector3d _velocityB = new Vector3d(0, 0, 0);
    private Vector3d _accelerationB = new Vector3d(0, 0, 0);
    private Vector3d _attitudeE = new Vector3d(0.1, 0.1, 0.5);
    private Vector3d _angularVelocityB = new Vector3d(0, 0, 0);
    private Vector3d _angularAccelerationB = new Vector3d(0, 0, 0);
    private Vector3d _thrustB = new Vector3d(0, 0, 0);
    private Vector3d _torqueB = new Vector3d(0, 0, 0);
    private double _mass = 1.0f;
    private double _inertiaXx = 4.856e-3f;
    private double _inertiaYy = 4.856e-3f;
    private double _inertiaZz = 8.801e-3f;
    private int _numberOfRotors = 4;
    Rotor[] _rotors = new Rotor[4];
    private double[] _armLengths = new double[4] { 0.1, 0.1, 0.1, 0.1 };
    private double[] _armAngles = new double[4] { Mathd.PI / 4.0, 3.0 * Mathd.PI / 4.0, 5.0 * Mathd.PI / 4.0, 7.0 * Mathd.PI / 4.0 };

    private Vector3d _gravityE = new Vector3d(0, 0, 9.81);
    private double _dT = 0.001;
    private PIDController _altitudeController = new PIDController(0.001, -1, -0.1, -0.05, -0.2, 0.0, 1.0);
    private PIDController _rollController = new PIDController(0.001, 0, 0.5, 0.05, 0.1, -1.0, 1.0);
    private PIDController _pitchController = new PIDController(0.001, 0, 0.5, 0.1, 0.1, -1.0, 1.0);
    private PIDController _yawController = new PIDController(0.001, 0, 0.5, 0.05, 0.1, -1.0, 1.0);

    private readonly object _stateLock = new object();


    // void Start()
    // {
    //     Physics.autoSimulation = false;
    //     _rotors[0] = new Rotor(true, 0.05, 3.357e-5, 0.0000001984, 0.000000003733, 0.098, 6432, 1779);
    //     _rotors[1] = new Rotor(false, 0.05, 3.357e-5, 0.0000001984, 0.000000003733, 0.098, 6432, 1779);
    //     _rotors[2] = new Rotor(true, 0.05, 3.357e-5, 0.0000001984, 0.000000003733, 0.098, 6432, 1779);
    //     _rotors[3] = new Rotor(false, 0.05, 3.357e-5, 0.0000001984, 0.000000003733, 0.098, 6432, 1779);
    // }

    // Update is called once per frame

    // void FixedUpdate()
    // {
    //     Step(_dT);
    //     double altitudeThrottleModifier = _altitudeController.Calculate(_positionE.z);
    //     _rotors[0].SetThrottle(altitudeThrottleModifier);
    //     _rotors[1].SetThrottle(altitudeThrottleModifier);
    //     _rotors[2].SetThrottle(altitudeThrottleModifier);
    //     _rotors[3].SetThrottle(altitudeThrottleModifier);
    // }

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
        // double altitudeThrottleModifier = _altitudeController.Calculate(_positionE.z);
        // _rotors[0].SetThrottle(altitudeThrottleModifier);
        // _rotors[1].SetThrottle(altitudeThrottleModifier);
        // _rotors[2].SetThrottle(altitudeThrottleModifier);
        // _rotors[3].SetThrottle(altitudeThrottleModifier);
        QuadMixer(_altitudeController.Calculate(_positionE.z), _rollController.Calculate(_attitudeE.x), _pitchController.Calculate(_attitudeE.y), _yawController.Calculate(_attitudeE.z));
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
        _accelerationB = (_thrustB / _mass) + EarthToBody(_gravityE, _attitudeE);
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

    public Vector3d getPosition()
    {
        return _positionE;
    }

    public Vector3d getAttitude()
    {
        return _attitudeE;
    }
}