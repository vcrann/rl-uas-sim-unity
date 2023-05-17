using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class Multirotor : MonoBehaviour
{
    // Start is called before the first frame update
    private Vector3 _positionE = new Vector3(0, 0, 0);
    private Vector3 _velocityB = new Vector3(0, 0, 0);
    private Vector3 _accelerationB = new Vector3(0, 0, 0);
    private Vector3 _attitudeE = new Vector3(0, 0, 0);
    private Vector3 _angularVelocityB = new Vector3(0, 0, 0);
    private Vector3 _angularAccelerationB = new Vector3(0, 0, 0);
    private Vector3 _thrustB = new Vector3(0, 0, 0);
    private Vector3 _torqueB = new Vector3(0, 0, 0);
    private float _mass = 1.0f;
    private float _inertiaXx = 4.856e-3f;
    private float _inertiaYy = 4.856e-3f;
    private float _inertiaZz = 8.801e-3f;
    private int _numberOfRotors = 4;
    Rotor[] _rotors = new Rotor[4];
    private float[] _armLengths = new float[4] { 0.1f, 0.1f, 0.1f, 0.1f };
    private float[] _armAngles = new float[4] { Mathf.PI / 4.0f, 3.0f * Mathf.PI / 4.0f, 5.0f * Mathf.PI / 4.0f, 7.0f * Mathf.PI / 4.0f };

    private Vector3 _gravityE = new Vector3(0, 0, 9.81f);


    void Start()
    {
        Physics.autoSimulation = false;
        Time.fixedDeltaTime = 0.001f;
        _rotors[0] = new Rotor(true, 0.05f, 3.357e-5f, 0.0000001984f, 0.000000003733f, 0.098f, 6432f, 1779f);
        _rotors[1] = new Rotor(false, 0.05f, 3.357e-5f, 0.0000001984f, 0.000000003733f, 0.098f, 6432f, 1779f);
        _rotors[2] = new Rotor(true, 0.05f, 3.357e-5f, 0.0000001984f, 0.000000003733f, 0.098f, 6432f, 1779f);
        _rotors[3] = new Rotor(false, 0.05f, 3.357e-5f, 0.0000001984f, 0.000000003733f, 0.098f, 6432f, 1779f);

        _rotors[0].SetThrottle(0.279f);
        _rotors[1].SetThrottle(0.279f);
        _rotors[2].SetThrottle(0.281f);
        _rotors[3].SetThrottle(0.281f);
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = new Vector3(_positionE.y, -_positionE.z, _positionE.x);
        transform.eulerAngles = new Vector3(-_attitudeE.y * 180.0f / Mathf.PI, _attitudeE.z * 180.0f / Mathf.PI, -_attitudeE.x * 180.0f / Mathf.PI);
    }

    void FixedUpdate()
    {
        Step(Time.fixedDeltaTime);
    }

    void Step(float dT)
    {
        // Calculate the thrust and torque
        _thrustB = new Vector3(0, 0, 0);
        _torqueB = new Vector3(0, 0, 0);
        for (int i = 0; i <= _numberOfRotors - 1; i++)
        {
            _rotors[i].Step(dT);
            _thrustB.z -= _rotors[i].GetThrust(); // thrust upwards is negative in body frame, can only ever be upwards
            _torqueB += new Vector3(-_armLengths[i] * _rotors[i].GetThrust() * Mathf.Sin(_armAngles[i]), _armLengths[i] * _rotors[i].GetThrust() * Mathf.Cos(_armAngles[i]), _rotors[i].GetTorque());
            _torqueB += _rotors[i].GetGyroscopicTorque(_angularVelocityB);
        }

        // Linear
        _accelerationB = (_thrustB / _mass) + EarthToBody(_gravityE, _attitudeE);
        _velocityB += _accelerationB * dT;
        _positionE += BodyToEarth(_velocityB, _attitudeE) * dT;

        // Angular
        Vector3 _inertia = new Vector3(_inertiaXx, _inertiaYy, _inertiaZz);
        Vector3 _unscaledAngularAccelerationB = (-Vector3.Cross(_angularVelocityB, Vector3.Scale(_inertia, _angularVelocityB)) + _torqueB);
        _angularAccelerationB.x = _unscaledAngularAccelerationB.x / _inertia.x;
        _angularAccelerationB.y = _unscaledAngularAccelerationB.y / _inertia.y;
        _angularAccelerationB.z = _unscaledAngularAccelerationB.z / _inertia.z;

        Debug.Log(_angularAccelerationB);

        _angularVelocityB += _angularAccelerationB * dT;
        _attitudeE += _angularVelocityB * dT;
    }

    Vector3 BodyToEarth(Vector3 vectorB, Vector3 rotationB)
    {
        float x = vectorB.x;
        float y = vectorB.y;
        float z = vectorB.z;
        float phi = rotationB.x;
        float theta = rotationB.y;
        float psi = rotationB.z;

        float[,] rotationMatrixBToE = new float[3, 3];
        rotationMatrixBToE[0, 0] = Mathf.Cos(psi) * Mathf.Cos(theta);
        rotationMatrixBToE[0, 1] = Mathf.Cos(psi) * Mathf.Sin(theta) * Mathf.Sin(phi) - Mathf.Sin(psi) * Mathf.Cos(phi);
        rotationMatrixBToE[0, 2] = Mathf.Cos(psi) * Mathf.Sin(theta) * Mathf.Cos(phi) + Mathf.Sin(psi) * Mathf.Sin(phi);
        rotationMatrixBToE[1, 0] = Mathf.Sin(psi) * Mathf.Cos(theta);
        rotationMatrixBToE[1, 1] = Mathf.Sin(psi) * Mathf.Sin(theta) * Mathf.Sin(phi) + Mathf.Cos(psi) * Mathf.Cos(phi);
        rotationMatrixBToE[1, 2] = Mathf.Sin(psi) * Mathf.Sin(theta) * Mathf.Cos(phi) - Mathf.Cos(psi) * Mathf.Sin(phi);
        rotationMatrixBToE[2, 0] = -Mathf.Sin(theta);
        rotationMatrixBToE[2, 1] = Mathf.Cos(theta) * Mathf.Sin(phi);
        rotationMatrixBToE[2, 2] = Mathf.Cos(theta) * Mathf.Cos(phi);

        Vector3 result = new Vector3(
            rotationMatrixBToE[0, 0] * x + rotationMatrixBToE[0, 1] * y + rotationMatrixBToE[0, 2] * z,
            rotationMatrixBToE[1, 0] * x + rotationMatrixBToE[1, 1] * y + rotationMatrixBToE[1, 2] * z,
            rotationMatrixBToE[2, 0] * x + rotationMatrixBToE[2, 1] * y + rotationMatrixBToE[2, 2] * z
        );
        return result;
    }

    // method to convert coordinates from earth frame to body frame
    Vector3 EarthToBody(Vector3 vectorE, Vector3 rotationB)
    {
        float x = vectorE.x;
        float y = vectorE.y;
        float z = vectorE.z;
        float phi = rotationB.x;
        float theta = rotationB.y;
        float psi = rotationB.z;

        float[,] rotationMatrixEToB = new float[3, 3];
        rotationMatrixEToB[0, 0] = Mathf.Cos(psi) * Mathf.Cos(theta);
        rotationMatrixEToB[0, 1] = Mathf.Sin(psi) * Mathf.Cos(theta);
        rotationMatrixEToB[0, 2] = -Mathf.Sin(theta);
        rotationMatrixEToB[1, 0] = Mathf.Cos(psi) * Mathf.Sin(theta) * Mathf.Sin(phi) - Mathf.Sin(psi) * Mathf.Cos(phi);
        rotationMatrixEToB[1, 1] = Mathf.Sin(psi) * Mathf.Sin(theta) * Mathf.Sin(phi) + Mathf.Cos(psi) * Mathf.Cos(phi);
        rotationMatrixEToB[1, 2] = Mathf.Cos(theta) * Mathf.Sin(phi);
        rotationMatrixEToB[2, 0] = Mathf.Cos(psi) * Mathf.Sin(theta) * Mathf.Cos(phi) + Mathf.Sin(psi) * Mathf.Sin(phi);
        rotationMatrixEToB[2, 1] = Mathf.Sin(psi) * Mathf.Sin(theta) * Mathf.Cos(phi) - Mathf.Cos(psi) * Mathf.Sin(phi);
        rotationMatrixEToB[2, 2] = Mathf.Cos(theta) * Mathf.Cos(phi);

        Vector3 result = new Vector3(
            rotationMatrixEToB[0, 0] * x + rotationMatrixEToB[0, 1] * y + rotationMatrixEToB[0, 2] * z,
            rotationMatrixEToB[1, 0] * x + rotationMatrixEToB[1, 1] * y + rotationMatrixEToB[1, 2] * z,
            rotationMatrixEToB[2, 0] * x + rotationMatrixEToB[2, 1] * y + rotationMatrixEToB[2, 2] * z
        );
        return result;
    }
}
