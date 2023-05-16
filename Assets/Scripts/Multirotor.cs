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
    private float _mass = 0.468f;
    private float _inertiaXx = 4.856e-3f;
    private float _inertiaYy = 4.856e-3f;
    private float _inertiaZz = 8.801e-3f;
    private float _numberOfRotors = 4f;
    private Rotor[] _rotors;
    private float[] _armLengths = new float[4] { 0.1f, 0.1f, 0.1f, 0.1f };
    private float[] _armAngles = new float[4] { Mathf.PI / 4.0f, 3.0f * Mathf.PI / 4.0f, 5.0f * Mathf.PI / 4.0f, 7.0f * Mathf.PI / 4.0f };


    void Start()
    {
        Physics.autoSimulation = false;
        for (int i = 0; i <= 4; i++)
        {
            _rotors[i] = new Rotor();
        }
    }

    // Update is called once per frame
    void Update()
    {
        //transform.position = new Vector3(transform.position.x, transform.position.y + 0.02f, transform.position.z + 0.01f);
        //Physics.Simulate(Time.fixedDeltaTime);
    }

    void Step(float dT)
    {
        // Calculate the thrust and torque
        _thrustB = new Vector3(0, 0, 0);
        _torqueB = new Vector3(0, 0, 0);
        for (int i = 0; i <= _numberOfRotors; i++)
        {
            _rotors[i].Step(dT);
            _thrustB.z -= _rotors[i].GetThrust(); // thrust upwards is negative in body frame, can only ever be upwards
            _torqueB.z += _rotors[i].GetTorque();
            _torqueB += _rotors[i].GetGyroscopicTorque(_angularVelocityB);
        }

        // Calculate the acceleration
        _accelerationB = _thrustB / _mass;
        _velocityB += _accelerationB * dT;
        _positionE += BodyToEarth(_velocityB, _attitudeE) * dT;

        // Calculate the angular acceleration
        //_angularAccelerationB = (-_angular_velocity_b.cross(Eigen::Vector3d(_inertia_xx, _inertia_yy, _inertia_zz) * _angular_velocity_b) + _torque_b).array() / Eigen::Vector3d(_inertia_xx, _inertia_yy, _inertia_zz).array()
        Vector3 _inertia = new Vector3(_inertiaXx, _inertiaYy, _inertiaZz);
        Vector3 _unscaledAngularAccelerationB = (-Vector3.Cross(_angularVelocityB, Vector3.Scale(_inertia, _angularVelocityB)) + _torqueB);
        _angularAccelerationB.x = _unscaledAngularAccelerationB.x / _inertia.x;
        _angularAccelerationB.y = _unscaledAngularAccelerationB.y / _inertia.y;
        _angularAccelerationB.z = _unscaledAngularAccelerationB.z / _inertia.z;

        // Calculate the velocity
        _velocityB += _accelerationB * dT;
        _angularVelocityB += _angularAccelerationB * dT;

        // Calculate the position
        _positionE += BodyToEarth(_velocityB, _attitudeE) * dT;

        // Calculate the attitude
        _attitudeE += BodyToEarth(_angularVelocityB, _attitudeE) * dT;
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
}
