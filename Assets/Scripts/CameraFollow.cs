using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform targetObject;
    private Vector3 initialOffset;
    private Vector3 cameraPosition;
    private Vector3 cameraVelocity;
    // Start is called before the first frame update
    void Start()
    {
        targetObject = GameObject.Find("spinningPropsDrone").transform;
        initialOffset = transform.position - targetObject.position;
    }

    // Update is called once per frame
    void Update()
    {
        cameraPosition = targetObject.position + initialOffset;
        transform.position = Vector3.SmoothDamp(transform.position, cameraPosition, ref cameraVelocity, 0.1f);
    }
}
