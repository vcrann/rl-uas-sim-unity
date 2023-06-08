using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MainCameraFollow : MonoBehaviour
{
    public GameObject target;
    public float speed = 2;

    Vector3 offset;

    void Start()
    {
        offset = target.transform.position - transform.position;
    }

    void LateUpdate()
    {
        // Look
        //var newRotation = Quaternion.LookRotation(target.transform.position - transform.position);

        //transform.rotation = Quaternion.Slerp(transform.rotation, newRotation, speed * Time.deltaTime);

        // Move
        // Vector3 newPosition = target.transform.position - target.transform.forward * offset.z - target.transform.up * offset.y;
        // transform.position = Vector3.Slerp(transform.position, newPosition, Time.deltaTime * speed);

        
        
        // Rotate the camera every frame so it keeps looking at the target
        transform.LookAt(target.transform);

        // Same as above, but setting the worldUp parameter to Vector3.left in this example turns the camera on its side
        transform.LookAt(target.transform, Vector3.up);
    }
}