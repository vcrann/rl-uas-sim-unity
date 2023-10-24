using System.Collections;
using System.Collections.Generic;
using System.Threading;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.VisualScripting;

public class MultirotorAgent : Agent
{
    private MultirotorDynamics _multirotorDynamics = new MultirotorDynamics();
    private Thread modelThread;
    public GameObject goalSphere;
    public Transform[] rotorObjects = new Transform[4];
    Plane plane = new Plane(Vector3.up, 0);
    public Camera overheadCamera;

    public float flyingAreaHeight = 10.0f;
    public float flyingAreaWidth = 10.0f;
    public float flyingAreaDepth = 10.0f;
    float maxGoalDistance = 0.0f;
    public float maxSpeed = 20.0f;
    public float maxAngularSpeed = 20.0f;

    private Vector3 goalLocation = new(0, 0, 0);

    public override void Initialize()
    {
        if (!Academy.Instance.IsCommunicatorOn)
        {
            this.MaxStep = 1000;
        }
    }

    // Start is called before the first frame update
    void Start()
    {
        //Physics.autoSimulation = false;
        rotorObjects[0] = GameObject.Find("Rotor01").transform;
        rotorObjects[1] = GameObject.Find("Rotor02").transform;
        rotorObjects[2] = GameObject.Find("Rotor03").transform;
        rotorObjects[3] = GameObject.Find("Rotor04").transform;
        _multirotorDynamics.SetupRotors();
        maxGoalDistance = Mathf.Sqrt(flyingAreaHeight * flyingAreaHeight + flyingAreaWidth * flyingAreaWidth + flyingAreaDepth * flyingAreaDepth);
        // modelThread = new Thread(_multirotorDynamics.RunModel);
        // modelThread.Start();
    }

    public override void OnEpisodeBegin()
    {
        goalLocation = new(Random.Range(-flyingAreaDepth / 2, flyingAreaDepth / 2), Random.Range(-flyingAreaWidth / 2, flyingAreaWidth / 2), Random.Range(-flyingAreaHeight, 0));
        goalSphere.transform.position = new Vector3(goalLocation.y, -goalLocation.z, goalLocation.x);
        //zero states
        _multirotorDynamics.Reset();
        transform.position = new Vector3((float)_multirotorDynamics.GetPosition().y, -(float)_multirotorDynamics.GetPosition().z, (float)_multirotorDynamics.GetPosition().x);
        transform.eulerAngles = new Vector3(-(float)_multirotorDynamics.GetAttitude().y * 180.0f / Mathf.PI, (float)_multirotorDynamics.GetAttitude().z * 180.0f / Mathf.PI, -(float)_multirotorDynamics.GetAttitude().x * 180.0f / Mathf.PI);
    }

    public override void CollectObservations(VectorSensor sensor)
    {

        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetPosition().x / (flyingAreaDepth / 2), -1, 1));
        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetPosition().y / (flyingAreaWidth / 2), -1, 1));
        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetPosition().z / (flyingAreaHeight / 2) - 1, -1, 1));

        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetAttitude().x / Mathf.PI, -1, 1));
        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetAttitude().y / Mathf.PI, -1, 1));
        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetAttitude().z / Mathf.PI, -1, 1));

        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetVelocity().x / maxSpeed, -1, 1));
        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetVelocity().y / maxSpeed, -1, 1));
        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetVelocity().z / maxSpeed, -1, 1));

        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetAngularVelocity().x / maxAngularSpeed, -1, 1));
        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetAngularVelocity().y / maxAngularSpeed, -1, 1));
        sensor.AddObservation(Mathf.Clamp((float)_multirotorDynamics.GetAngularVelocity().z / maxAngularSpeed, -1, 1));

        sensor.AddObservation(Mathf.Clamp(((float)_multirotorDynamics.GetPosition().x - goalLocation.x) / flyingAreaDepth, -1, 1));
        sensor.AddObservation(Mathf.Clamp(((float)_multirotorDynamics.GetPosition().y - goalLocation.y) / flyingAreaWidth, -1, 1));
        sensor.AddObservation(Mathf.Clamp(((float)_multirotorDynamics.GetPosition().z - goalLocation.z) / flyingAreaHeight, -1, 1));
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // Actions, size = 4
        //each element converted from -1 to 1 to 0 to 1
        double[] rotorThrottles = { actionBuffers.ContinuousActions[0] / 2 + 0.5, actionBuffers.ContinuousActions[1] / 2 + 0.5, actionBuffers.ContinuousActions[2] / 2 + 0.5, actionBuffers.ContinuousActions[3] / 2 + 0.5 };
        _multirotorDynamics.SetRotorThrottles(rotorThrottles);

        // Reward for distance to target
        float normalisedDistanceToTarget = Mathf.Clamp(Vector3.Distance((Vector3)_multirotorDynamics.GetPosition(), goalLocation) / maxGoalDistance, 0, 1);
        AddReward(Mathf.Exp(-5 * normalisedDistanceToTarget));

        // Reward for not yawing like crazy
        AddReward(-Mathf.Clamp((float)_multirotorDynamics.GetAngularVelocity().z / maxAngularSpeed, -1, 1));

        if (Vector3.Dot(transform.up, Vector3.down) > 0)
        {
            AddReward(-100.0f);
            EndEpisode();
        }

        if (_multirotorDynamics.GetPosition().z > 0 | _multirotorDynamics.GetPosition().z < -flyingAreaHeight | _multirotorDynamics.GetPosition().x > flyingAreaDepth / 2 | _multirotorDynamics.GetPosition().x < -flyingAreaDepth / 2 | _multirotorDynamics.GetPosition().y > flyingAreaWidth / 2 | _multirotorDynamics.GetPosition().y < -flyingAreaWidth / 2)
        {
            AddReward(-100.0f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = -1.0f;
        continuousActionsOut[1] = -1.0f;
        continuousActionsOut[2] = -1.0f;
        continuousActionsOut[3] = -1.0f;
    }
    void Update()
    {
        transform.position = new Vector3((float)_multirotorDynamics.GetPosition().y, -(float)_multirotorDynamics.GetPosition().z, (float)_multirotorDynamics.GetPosition().x);
        transform.eulerAngles = new Vector3(-(float)_multirotorDynamics.GetAttitude().y * 180.0f / Mathf.PI - 180, (float)_multirotorDynamics.GetAttitude().z * 180.0f / Mathf.PI - 180, -(float)_multirotorDynamics.GetAttitude().x * 180.0f / Mathf.PI - 180);

        //just rotors 2 and 3 for now
        //TODO check directions
        for (int i = 0; i < 4; i++)
        {
            rotorObjects[i].Rotate(0.0f, (float)_multirotorDynamics.GetRotorSpeeds()[i] * 60 * 360.0f / Mathf.PI * Time.deltaTime, 0.0f);
        }
    }

    void FixedUpdate()
    {
        _multirotorDynamics.Step(Time.fixedDeltaTime);
    }
}