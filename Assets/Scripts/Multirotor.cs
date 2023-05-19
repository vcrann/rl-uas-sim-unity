using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
public class Multirotor : MonoBehaviour
{
    private MultirotorDynamics _multirotorDynamics = new MultirotorDynamics();
    private Thread modelThread;
    public Transform[] rotorObjects = new Transform[4];

    // Start is called before the first frame update
    void Start()
    {
        Physics.autoSimulation = false;
        rotorObjects[1] = GameObject.Find("Rotor02").transform;
        rotorObjects[2] = GameObject.Find("Rotor03").transform;
        modelThread = new Thread(_multirotorDynamics.RunModel);
        modelThread.Start();
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = new Vector3((float)_multirotorDynamics.GetPosition().y, -(float)_multirotorDynamics.GetPosition().z, (float)_multirotorDynamics.GetPosition().x);
        transform.eulerAngles = new Vector3(-(float)_multirotorDynamics.GetAttitude().y * 180.0f / Mathf.PI, (float)_multirotorDynamics.GetAttitude().z * 180.0f / Mathf.PI, -(float)_multirotorDynamics.GetAttitude().x * 180.0f / Mathf.PI);

        float horizontalInput = Input.GetAxis("Horizontal");
        float forwardInput = Input.GetAxis("Forward");
        float yawInput = Input.GetAxis("Yaw");
        float altitudeInput = Input.GetAxis("Vertical");
        _multirotorDynamics.GetManualInput(horizontalInput, forwardInput, yawInput, altitudeInput);

        //just rotors 2 and 3 for now
        //TODO check directions
        for (int i = 1; i < 3; i++)
        {
            rotorObjects[i].Rotate(0.0f, (float)_multirotorDynamics.GetRotorSpeeds()[i] * 60 * 360.0f / Mathf.PI * Time.deltaTime, 0.0f);
        }
    }
}
