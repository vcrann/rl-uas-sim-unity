using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
public class Multirotor : MonoBehaviour
{
    private MultirotorDynamics _multirotorDynamics = new MultirotorDynamics();
    private Thread modelThread;
    public Transform[] rotorObjects = new Transform[4];
    Plane plane = new Plane(Vector3.up, 0);
    public Camera overheadCamera;
    private int _flightMode = 0;
    // Start is called before the first frame update
    void Start()
    {
        Physics.autoSimulation = false;
        rotorObjects[0] = GameObject.Find("Rotor01").transform;
        rotorObjects[1] = GameObject.Find("Rotor02").transform;
        rotorObjects[2] = GameObject.Find("Rotor03").transform;
        rotorObjects[3] = GameObject.Find("Rotor04").transform;
        modelThread = new Thread(_multirotorDynamics.RunModel);
        modelThread.Start();
    }

    // Update is called once per frame
    void Update()
    {
        //Debug.Log("Position: " + _multirotorDynamics.GetPosition().ToString());
        transform.position = new Vector3((float)_multirotorDynamics.GetPosition().y, -(float)_multirotorDynamics.GetPosition().z, (float)_multirotorDynamics.GetPosition().x);
        transform.eulerAngles = new Vector3(-(float)_multirotorDynamics.GetAttitude().y * 180.0f / Mathf.PI, (float)_multirotorDynamics.GetAttitude().z * 180.0f / Mathf.PI, -(float)_multirotorDynamics.GetAttitude().x * 180.0f / Mathf.PI);

        //just rotors 2 and 3 for now
        //TODO check directions
        for (int i = 0; i < 4; i++)
        {
            rotorObjects[i].Rotate(0.0f, (float)_multirotorDynamics.GetRotorSpeeds()[i] * 60 * 360.0f / Mathf.PI * Time.deltaTime, 0.0f);
        }

        switch (_flightMode)
        {
            case 0:
                float horizontalInput = Input.GetAxis("Horizontal");
                float forwardInput = Input.GetAxis("Forward");
                float yawInput = Input.GetAxis("Yaw");
                float altitudeInput = Input.GetAxis("Vertical");
                _multirotorDynamics.SetManualInput(horizontalInput, forwardInput, yawInput, altitudeInput);
                break;
            case 1:
                if (Input.GetMouseButtonDown(0))
                {
                    // used to check if the mouse is over the overhead camera view, otherwise interacting with ui elements will cause the drone to move
                    Vector3 viewportMousePosition = overheadCamera.ScreenToViewportPoint(Input.mousePosition);
                    if (viewportMousePosition.x >= 0 && viewportMousePosition.x <= 1 && viewportMousePosition.y >= 0 && viewportMousePosition.y <= 1)
                    {
                        Vector3 mousePosition = GetMousePosition();
                        _multirotorDynamics.SetDesiredPosition(new Vector3(mousePosition.z, mousePosition.x, -mousePosition.y));
                    }
                }
                break;
        }
    }

    Vector3 GetMousePosition()
    {
        float distanceToPlane;
        Vector3 worldMousePosition = new Vector3();
        Ray ray = overheadCamera.ScreenPointToRay(Input.mousePosition);
        if (plane.Raycast(ray, out distanceToPlane))
        {
            worldMousePosition = ray.GetPoint(distanceToPlane);
        }
        return worldMousePosition;
    }

    public void SetFlightMode(int flightMode)
    {
        _flightMode = flightMode;
        _multirotorDynamics.SetFlightMode(flightMode);
        Debug.Log("Flight mode set to " + flightMode);
    }
}
