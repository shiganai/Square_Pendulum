using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class rodrigues_Rotation : MonoBehaviour
{
    public float alpha, beta, gamma;
    public float dalpha, dbeta, dgamma;
    private Matrix r_Shoulder, l_Shoulder, r_Hip, l_Hip;

    private GameObject r_Shoulder_Go, l_Shoulder_Go, r_Hip_Go, l_Hip_Go;

    public Matrix ax_Vec_X, ax_Vec_Y, ax_Vec_Z;

    // Start is called before the first frame update
    void Start()
    {
        r_Shoulder_Go = GameObject.Find("r_Shoulder");
        l_Shoulder_Go = GameObject.Find("l_Shoulder");
        r_Hip_Go = GameObject.Find("r_Hip");
        l_Hip_Go = GameObject.Find("l_Hip");

        ax_Vec_X = NMC.convert_Vector3_To_Matrix(new Vector3(1, 0, 0));
        ax_Vec_Y = NMC.convert_Vector3_To_Matrix(new Vector3(0, 1, 0));
        ax_Vec_Z = NMC.convert_Vector3_To_Matrix(new Vector3(0, 0, 1));

        r_Shoulder = Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { 0.5f, 1, 0 }
        }));
        l_Shoulder = Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { -0.5f, 1, 0 }
        }));
        r_Hip = Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { 0.5f, -1, 0 }
        }));
        l_Hip = Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { -0.5f, -1, 0 }
        }));
    }

    // Update is called once per frame
    void Update()
    {

        var rodriguess_Alpha = new Matrix(new float[,]
        {
            { ax_Vec_X[0,0] * ax_Vec_X[0,0] * (1-cos(dalpha * Time.fixedDeltaTime)) + cos(dalpha * Time.fixedDeltaTime),
                ax_Vec_X[0,0] * ax_Vec_X[1,0] * (1-cos(dalpha * Time.fixedDeltaTime)) - ax_Vec_X[2,0] * sin(dalpha * Time.fixedDeltaTime),
                ax_Vec_X[0,0] * ax_Vec_X[2,0] * (1-cos(dalpha * Time.fixedDeltaTime)) + ax_Vec_X[1,0] * sin(dalpha * Time.fixedDeltaTime) },
            { ax_Vec_X[0,0] * ax_Vec_X[1,0] * (1-cos(dalpha * Time.fixedDeltaTime)) + ax_Vec_X[2,0] *  sin(dalpha * Time.fixedDeltaTime),
                ax_Vec_X[1,0] * ax_Vec_X[1,0] * (1-cos(dalpha * Time.fixedDeltaTime)) + cos(dalpha * Time.fixedDeltaTime),
                ax_Vec_X[1,0] * ax_Vec_X[2,0] * (1-cos(dalpha * Time.fixedDeltaTime)) - ax_Vec_X[0,0] * sin(dalpha * Time.fixedDeltaTime) },
            { ax_Vec_X[0,0] * ax_Vec_X[2,0] * (1-cos(dalpha * Time.fixedDeltaTime)) - ax_Vec_X[1,0] *  sin(dalpha * Time.fixedDeltaTime),
                ax_Vec_X[1,0] * ax_Vec_X[2,0] * (1-cos(dalpha * Time.fixedDeltaTime)) + ax_Vec_X[0,0] * sin(dalpha * Time.fixedDeltaTime),
                ax_Vec_X[2,0] * ax_Vec_X[2,0] * (1-cos(dalpha * Time.fixedDeltaTime)) + cos(dalpha * Time.fixedDeltaTime) },
        });
        r_Shoulder = rodriguess_Alpha * r_Shoulder;
        l_Shoulder = rodriguess_Alpha * l_Shoulder;
        r_Hip = rodriguess_Alpha * r_Hip;
        l_Hip = rodriguess_Alpha * l_Hip;
        ax_Vec_X = rodriguess_Alpha * ax_Vec_X;
        ax_Vec_Y = rodriguess_Alpha * ax_Vec_Y;
        ax_Vec_Z = rodriguess_Alpha * ax_Vec_Z;

        var rodriguess_Beta = new Matrix(new float[,]
        {
            { ax_Vec_Y[0,0] * ax_Vec_Y[0,0] * (1-cos(dbeta * Time.fixedDeltaTime)) + cos(dbeta * Time.fixedDeltaTime),
                ax_Vec_Y[0,0] * ax_Vec_Y[1,0] * (1-cos(dbeta * Time.fixedDeltaTime)) - ax_Vec_Y[2,0] * sin(dbeta * Time.fixedDeltaTime),
                ax_Vec_Y[0,0] * ax_Vec_Y[2,0] * (1-cos(dbeta * Time.fixedDeltaTime)) + ax_Vec_Y[1,0] * sin(dbeta * Time.fixedDeltaTime) },
            { ax_Vec_Y[0,0] * ax_Vec_Y[1,0] * (1-cos(dbeta * Time.fixedDeltaTime)) + ax_Vec_Y[2,0] *  sin(dbeta * Time.fixedDeltaTime),
                ax_Vec_Y[1,0] * ax_Vec_Y[1,0] * (1-cos(dbeta * Time.fixedDeltaTime)) + cos(dbeta * Time.fixedDeltaTime),
                ax_Vec_Y[1,0] * ax_Vec_Y[2,0] * (1-cos(dbeta * Time.fixedDeltaTime)) - ax_Vec_Y[0,0] * sin(dbeta * Time.fixedDeltaTime) },
            { ax_Vec_Y[0,0] * ax_Vec_Y[2,0] * (1-cos(dbeta * Time.fixedDeltaTime)) - ax_Vec_Y[1,0] *  sin(dbeta * Time.fixedDeltaTime),
                ax_Vec_Y[1,0] * ax_Vec_Y[2,0] * (1-cos(dbeta * Time.fixedDeltaTime)) + ax_Vec_Y[0,0] * sin(dbeta * Time.fixedDeltaTime),
                ax_Vec_Y[2,0] * ax_Vec_Y[2,0] * (1-cos(dbeta * Time.fixedDeltaTime)) + cos(dbeta * Time.fixedDeltaTime) },
        });
        r_Shoulder = rodriguess_Beta * r_Shoulder;
        l_Shoulder = rodriguess_Beta * l_Shoulder;
        r_Hip = rodriguess_Beta * r_Hip;
        l_Hip = rodriguess_Beta * l_Hip;
        ax_Vec_X = rodriguess_Beta * ax_Vec_X;
        ax_Vec_Y = rodriguess_Beta * ax_Vec_Y;
        ax_Vec_Z = rodriguess_Beta * ax_Vec_Z;

        var rodriguess_Gamma = new Matrix(new float[,]
        {
            { ax_Vec_Z[0,0] * ax_Vec_Z[0,0] * (1-cos(dgamma * Time.fixedDeltaTime)) + cos(dgamma * Time.fixedDeltaTime),
                ax_Vec_Z[0,0] * ax_Vec_Z[1,0] * (1-cos(dgamma * Time.fixedDeltaTime)) - ax_Vec_Z[2,0] * sin(dgamma * Time.fixedDeltaTime),
                ax_Vec_Z[0,0] * ax_Vec_Z[2,0] * (1-cos(dgamma * Time.fixedDeltaTime)) + ax_Vec_Z[1,0] * sin(dgamma * Time.fixedDeltaTime) },
            { ax_Vec_Z[0,0] * ax_Vec_Z[1,0] * (1-cos(dgamma * Time.fixedDeltaTime)) + ax_Vec_Z[2,0] *  sin(dgamma * Time.fixedDeltaTime),
                ax_Vec_Z[1,0] * ax_Vec_Z[1,0] * (1-cos(dgamma * Time.fixedDeltaTime)) + cos(dgamma * Time.fixedDeltaTime),
                ax_Vec_Z[1,0] * ax_Vec_Z[2,0] * (1-cos(dgamma * Time.fixedDeltaTime)) - ax_Vec_Z[0,0] * sin(dgamma * Time.fixedDeltaTime) },
            { ax_Vec_Z[0,0] * ax_Vec_Z[2,0] * (1-cos(dgamma * Time.fixedDeltaTime)) - ax_Vec_Z[1,0] *  sin(dgamma * Time.fixedDeltaTime),
                ax_Vec_Z[1,0] * ax_Vec_Z[2,0] * (1-cos(dgamma * Time.fixedDeltaTime)) + ax_Vec_Z[0,0] * sin(dgamma * Time.fixedDeltaTime),
                ax_Vec_Z[2,0] * ax_Vec_Z[2,0] * (1-cos(dgamma * Time.fixedDeltaTime)) + cos(dgamma * Time.fixedDeltaTime) },
        });
        r_Shoulder = rodriguess_Gamma * r_Shoulder;
        l_Shoulder = rodriguess_Gamma * l_Shoulder;
        r_Hip = rodriguess_Gamma * r_Hip;
        l_Hip = rodriguess_Gamma * l_Hip;
        ax_Vec_X = rodriguess_Gamma * ax_Vec_X;
        ax_Vec_Y = rodriguess_Gamma * ax_Vec_Y;
        ax_Vec_Z = rodriguess_Gamma * ax_Vec_Z;

        r_Shoulder_Go.transform.position = NMC.convert_Matrix_To_Vector3(r_Shoulder);
        l_Shoulder_Go.transform.position = NMC.convert_Matrix_To_Vector3(l_Shoulder);
        r_Hip_Go.transform.position = NMC.convert_Matrix_To_Vector3(r_Hip);
        l_Hip_Go.transform.position = NMC.convert_Matrix_To_Vector3(l_Hip);

        alpha += dalpha * Time.fixedDeltaTime;
        beta += dbeta * Time.fixedDeltaTime;
        gamma += dgamma * Time.fixedDeltaTime;

        //transform.rotation = Quaternion.identity;

        //transform.Rotate(transform.right, dalpha * Mathf.Rad2Deg * Time.fixedDeltaTime);
        //transform.Rotate(transform.forward, dbeta * Mathf.Rad2Deg * Time.fixedDeltaTime);


        //transform.forward = NMC.convert_Matrix_To_Vector3(r_Shoulder - r_Hip);
        //transform.right = NMC.convert_Matrix_To_Vector3(r_Shoulder - l_Shoulder);
    }

    float cos(float theta)
    {
        return Mathf.Cos(theta);
    }

    float sin(float theta)
    {
        return Mathf.Sin(theta);
    }
}
