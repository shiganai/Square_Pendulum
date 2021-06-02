using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class move_Cube : MonoBehaviour
{
    public bool is_moved;
    public bool is_Rotated;

    public float rotate_Degree;
    private float rotate_Rad;
    // Start is called before the first frame update
    void Start()
    {
        is_moved = false;
        is_Rotated = false;
        rotate_Rad = rotate_Degree * Mathf.Deg2Rad;
    }

    // Update is called once per frame
    void Update()
    {
        if (is_moved)
        {
            is_moved = false;

            Matrix4x4 a = new Matrix4x4(new Vector4(1, 0, 0, 1), new Vector4(0, 1, 0, 0), new Vector4(0, 0, 0, 1), new Vector4(0, 0, 0, 1));

            Vector4 now_Position = new Vector4(transform.position.x, transform.position.y, transform.position.z, 1);

            Vector4 after_Position = new Vector4(
                now_Position.x * a.m00 + now_Position.y * a.m10 + now_Position.z * a.m20 + now_Position.w * a.m30,
                now_Position.x * a.m01 + now_Position.y * a.m11 + now_Position.z * a.m21 + now_Position.w * a.m31,
                now_Position.x * a.m02 + now_Position.y * a.m12 + now_Position.z * a.m22 + now_Position.w * a.m32,
                now_Position.x * a.m03 + now_Position.y * a.m13 + now_Position.z * a.m23 + now_Position.w * a.m33
                );

            Vector3 target_Position = new Vector3(after_Position.x, after_Position.y, after_Position.z);

            transform.position = target_Position;
        }
        if (is_Rotated)
        {
            is_Rotated = false;

            rotate_Rad = rotate_Degree * Mathf.Deg2Rad;

            float[,] b = new float[,] { { 1, 0, 0, 0 },
                { 0, Mathf.Cos(rotate_Rad), -Mathf.Sin(rotate_Rad), 0 },
                { 0, Mathf.Sin(rotate_Rad), Mathf.Cos(rotate_Rad), 0 },
                { 1, 0, 0, 0 } };

            float[] now_Position = new float[] { transform.forward.x, transform.forward.y, transform.forward.z, 1 };

            float[] after_Position = new float[]
            {
                now_Position[0] * b[0,0] + now_Position[1] * b[0,1] + now_Position[2] * b[0,2] + now_Position[3] * b[0,3],
                now_Position[0] * b[1,0] + now_Position[1] * b[1,1] + now_Position[2] * b[1,2] + now_Position[3] * b[1,3],
                now_Position[0] * b[2,0] + now_Position[1] * b[2,1] + now_Position[2] * b[2,2] + now_Position[3] * b[2,3],
                now_Position[0] * b[3,0] + now_Position[1] * b[3,1] + now_Position[2] * b[3,2] + now_Position[3] * b[3,3]
            };

            Vector3 target_Forward = new Vector3(after_Position[0], after_Position[1], after_Position[2]);

            transform.forward = target_Forward;
        }
    }
}
