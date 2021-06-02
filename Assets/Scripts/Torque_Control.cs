using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Torque_Control : MonoBehaviour
{
    private Rigidbody arm_L;
    private Rigidbody arm_R;
    private Rigidbody body;

    public drag_Control graph_Controler;

    public float torque_Body_L_Max;
    public float torque_Body_R_Max;
    // Start is called before the first frame update
    void Start()
    {
        arm_L = GameObject.Find("arm_L").GetComponent<Rigidbody>();
        arm_R = GameObject.Find("arm_R").GetComponent<Rigidbody>();
        body = GameObject.Find("body").GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void FixedUpdate()
    {

    }
}
