using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class add_Force : MonoBehaviour
{
    public Vector3 adding_Force;
    public Vector3 adding_Torque;
    public Rigidbody target_Rigidbody;
    // Start is called before the first frame update
    void Start()
    {
        target_Rigidbody = gameObject.GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        //target_Rigidbody.AddForce(transform.InverseTransformDirection(adding_Force));
        //target_Rigidbody.AddTorque(transform.InverseTransformDirection(adding_Torque));
        target_Rigidbody.AddForce(adding_Force);
        target_Rigidbody.AddTorque(adding_Torque);
    }
}
