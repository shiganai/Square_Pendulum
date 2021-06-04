using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Torque_Control : MonoBehaviour
{
    private GameObject arm_L;
    private GameObject arm_R;
    private GameObject body;

    private float width_Body;
    private float height_Body;
    private float depth_Body;
    private float m_Body;
    private float m_Hand;
    private float length_Hand;
    private float g;

    public float alpha_Body;
    public float beta_Body;
    public float gamma_Body;
    public float x_Head;
    public float y_Head;
    public float z_Head;

    public float r_Alpha_Hand;
    private float r_Beta_Hand;

    public float l_Alpha_Hand;
    private float l_Beta_Hand;

    private float dalpha_Body;
    private float dbeta_Body;
    private float dgamma_Body;
    private float dx_Head;
    private float dy_Head;
    private float dz_Head;

    private float dr_Alpha_Hand;
    private float dr_Beta_Hand;

    private float dl_Alpha_Hand;
    private float dl_Beta_Hand;

    private float r_X_Fixed, r_Y_Fixed, r_Z_Fixed;
    private float l_X_Fixed, l_Y_Fixed, l_Z_Fixed;

    public float r_F_X, r_F_Y, r_F_Z;
    private float l_F_X, l_F_Y, l_F_Z;

    //private float k;

    private Matrix r_P_Fixed;
    private Matrix l_P_Fixed;

    public drag_Control graph_Controler;

    public float torque_Body_L_Max;
    public float torque_Body_R_Max;

    public float dd_Threshold = 1e-1f;
    public bool toReset = false;

    public float r_Tau_Alpha_Shoulder, l_Tau_Alpha_Shoulder;
    public float r_Tau_Beta_Shoulder, l_Tau_Beta_Shoulder;


    // Start is called before the first frame update
    void Start()
    {
        arm_L = GameObject.Find("arm_L");
        arm_R = GameObject.Find("arm_R");
        body = GameObject.Find("body");

        Initialize();
        visualization();

    }

    public void Initialize()
    {
        width_Body = 1f;
        height_Body = 2f;
        depth_Body = 0.5f;
        m_Body = 10f;
        m_Hand = 1f;
        length_Hand = 1f;
        g = 1f;
        //k = 1e2f;

        alpha_Body = 0f * Mathf.Deg2Rad;
        beta_Body = 0;
        gamma_Body = 0;
        x_Head = 0f;
        y_Head = 1f;
        z_Head = 0f;

        r_Alpha_Hand = 0f * Mathf.Deg2Rad;
        r_Beta_Hand = 0;

        l_Alpha_Hand = 0f;
        l_Beta_Hand = 0;

        dalpha_Body = 0;
        dbeta_Body = 0;
        dgamma_Body = 0;
        dx_Head = 0f;
        dy_Head = 0f;
        dz_Head = 0f;

        dr_Alpha_Hand = 0;
        dr_Beta_Hand = 0;

        dl_Alpha_Hand = 0f;
        dl_Beta_Hand = 0;

        r_Tau_Alpha_Shoulder = 0;
        l_Tau_Alpha_Shoulder = 0;
        r_Tau_Beta_Shoulder = 0;
        l_Tau_Beta_Shoulder = 0;

        r_P_Fixed = NMC.convert_Vector3_To_Matrix(Vector3.zero);
        l_P_Fixed = NMC.convert_Vector3_To_Matrix(Vector3.zero);

        r_X_Fixed = r_P_Fixed[0, 0];
        r_Y_Fixed = r_P_Fixed[1, 0];
        r_Z_Fixed = r_P_Fixed[2, 0];

        l_X_Fixed = l_P_Fixed[0, 0];
        l_Y_Fixed = l_P_Fixed[1, 0];
        l_Z_Fixed = l_P_Fixed[2, 0];

        var r_Arm_Bottom = NMC.FRD_R_Arm_Bottom(length_Hand, r_Alpha_Hand, r_Beta_Hand, r_X_Fixed, r_Y_Fixed, r_Z_Fixed);
        var l_Arm_Bottom = NMC.FRD_L_Arm_Bottom(l_Alpha_Hand, l_Beta_Hand, l_X_Fixed, l_Y_Fixed, l_Z_Fixed, length_Hand);

        var r_Shoulder = NMC.FRD_R_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);
        var l_Shoulder = NMC.FRD_L_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);

        r_P_Fixed = r_Shoulder - r_Arm_Bottom;
        l_P_Fixed = l_Shoulder - l_Arm_Bottom;

        r_X_Fixed = r_P_Fixed[0, 0];
        r_Y_Fixed = r_P_Fixed[1, 0];
        r_Z_Fixed = r_P_Fixed[2, 0];

        l_X_Fixed = l_P_Fixed[0, 0];
        l_Y_Fixed = l_P_Fixed[1, 0];
        l_Z_Fixed = l_P_Fixed[2, 0];

        var head_P = (r_Shoulder + l_Shoulder) / 2;
        x_Head = head_P[0, 0];
        y_Head = head_P[1, 0];
        z_Head = head_P[2, 0];
    }

    // Update is called once per frame
    void FixedUpdate()
    {

        var r_Arm_Bottom = NMC.FRD_R_Arm_Bottom(length_Hand, r_Alpha_Hand, r_Beta_Hand, r_X_Fixed, r_Y_Fixed, r_Z_Fixed);
        var l_Arm_Bottom = NMC.FRD_L_Arm_Bottom(l_Alpha_Hand, l_Beta_Hand, l_X_Fixed, l_Y_Fixed, l_Z_Fixed, length_Hand);


        var head_P = (r_Arm_Bottom + l_Arm_Bottom) / 2;
        x_Head = head_P[0, 0];
        y_Head = head_P[1, 0];
        z_Head = head_P[2, 0];

        //var beta_Body_Tmp = NMC.find_Restrained_Beta_Body(alpha_Body, gamma_Body, l_Alpha_Hand, l_Beta_Hand, l_X_Fixed, l_Y_Fixed, length_Hand, r_Alpha_Hand, r_Beta_Hand, r_X_Fixed, r_Y_Fixed, width_Body);

        //if (!float.IsNaN(beta_Body_Tmp))
        //{
        //    beta_Body = beta_Body_Tmp;
        //}

        //find_Restrained_Velocity();

        find_F();

        //find_F_Spring();


        var dds_R_Hand = NMC.FFD_Dds_Arm_R(dr_Beta_Hand, dr_Alpha_Hand, g, length_Hand, m_Hand, r_Alpha_Hand, r_Beta_Hand, r_F_X, r_F_Y, r_F_Z, r_Tau_Beta_Shoulder, r_Tau_Alpha_Shoulder);
        var dds_L_Hand = NMC.FFD_Dds_Arm_L(dl_Beta_Hand, dl_Alpha_Hand, g, l_Alpha_Hand, l_Beta_Hand, l_F_X, l_F_Y, l_F_Z, l_Tau_Beta_Shoulder, l_Tau_Alpha_Shoulder, length_Hand, m_Hand);
        var dds_Body = NMC.FFD_Dds_Body(alpha_Body, beta_Body, dalpha_Body, depth_Body, dgamma_Body, g, gamma_Body, height_Body, l_Alpha_Hand, l_F_X, l_F_Y, l_F_Z, l_Tau_Beta_Shoulder, l_Tau_Alpha_Shoulder, m_Body, r_Alpha_Hand, r_F_X, r_F_Y, r_F_Z, r_Tau_Beta_Shoulder, r_Tau_Alpha_Shoulder, width_Body);

        if (Mathf.Abs(dds_R_Hand[1]) < dd_Threshold && dr_Beta_Hand == 0)
        {
            dds_R_Hand[1] = 0;
        }
        else
        {
            Debug.Log("become 3D");
        }

        if (Mathf.Abs(dds_L_Hand[1]) < dd_Threshold && dl_Beta_Hand == 0)
        {
            dds_L_Hand[1] = 0;
        }
        else
        {
            Debug.Log("become 3D");
        }

        if (Mathf.Abs(dds_Body[1]) < dd_Threshold && dbeta_Body == 0)
        {
            dds_Body[1] = 0;
        }
        else
        {
            Debug.Log("become 3D");
        }

        if (Mathf.Abs(dds_Body[2]) < dd_Threshold && dgamma_Body == 0)
        {
            dds_Body[2] = 0;
        }
        else
        {
            Debug.Log("become 3D");
        }

        r_Alpha_Hand += dr_Alpha_Hand * Time.fixedDeltaTime;
        r_Beta_Hand += dr_Beta_Hand * Time.fixedDeltaTime;
        l_Alpha_Hand += dl_Alpha_Hand * Time.fixedDeltaTime;
        l_Beta_Hand += dl_Beta_Hand * Time.fixedDeltaTime;

        alpha_Body += dalpha_Body * Time.fixedDeltaTime;
        beta_Body += dbeta_Body * Time.fixedDeltaTime;
        gamma_Body += dgamma_Body * Time.fixedDeltaTime;
        x_Head += dx_Head * Time.fixedDeltaTime;
        y_Head += dy_Head * Time.fixedDeltaTime;
        z_Head += dz_Head * Time.fixedDeltaTime;

        dr_Alpha_Hand += dds_R_Hand[0] * Time.fixedDeltaTime;
        dr_Beta_Hand += dds_R_Hand[1] * Time.fixedDeltaTime;
        dl_Alpha_Hand += dds_L_Hand[0] * Time.fixedDeltaTime;
        dl_Beta_Hand += dds_L_Hand[1] * Time.fixedDeltaTime;

        dalpha_Body += dds_Body[0] * Time.fixedDeltaTime;
        dbeta_Body += dds_Body[1] * Time.fixedDeltaTime;
        dgamma_Body += dds_Body[2] * Time.fixedDeltaTime;
        dx_Head += dds_Body[3] * Time.fixedDeltaTime;
        dy_Head += dds_Body[4] * Time.fixedDeltaTime;
        dz_Head += dds_Body[5] * Time.fixedDeltaTime;

        if (toReset)
        {
            Initialize();
            toReset = false;
        }

        visualization();
    }

    void visualization()
    {
        var r_Arm_Bottom = NMC.FRD_R_Arm_Bottom(length_Hand, r_Alpha_Hand, r_Beta_Hand, r_X_Fixed, r_Y_Fixed, r_Z_Fixed);
        var l_Arm_Bottom = NMC.FRD_L_Arm_Bottom(l_Alpha_Hand, l_Beta_Hand, l_X_Fixed, l_Y_Fixed, l_Z_Fixed, length_Hand);

        var r_Arm_Bottom_From_Fixed = r_Arm_Bottom - r_P_Fixed;
        var l_Arm_Bottom_From_Fixed = l_Arm_Bottom - l_P_Fixed;

        arm_R.transform.forward = NMC.convert_Matrix_To_Vector3(r_Arm_Bottom_From_Fixed);

        arm_R.transform.position = NMC.convert_Matrix_To_Vector3(r_Arm_Bottom_From_Fixed * 0.5f + r_P_Fixed);


        arm_L.transform.forward = NMC.convert_Matrix_To_Vector3(l_Arm_Bottom_From_Fixed);

        arm_L.transform.position = NMC.convert_Matrix_To_Vector3(l_Arm_Bottom_From_Fixed * 0.5f + l_P_Fixed);


        //var head_P = (r_Arm_Bottom + l_Arm_Bottom) / 2;
        //x_Head = head_P[0, 0];
        //y_Head = head_P[1, 0];
        //z_Head = head_P[2, 0];

        var r_Shoulder = NMC.FRD_R_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);
        var l_Shoulder = NMC.FRD_L_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);

        var r_Hip = NMC.FRD_R_Hip(alpha_Body, beta_Body, gamma_Body, height_Body, width_Body, x_Head, y_Head, z_Head);
        var l_Hip = NMC.FRD_L_Hip(alpha_Body, beta_Body, gamma_Body, height_Body, width_Body, x_Head, y_Head, z_Head);

        var vec_R_L = r_Shoulder - l_Shoulder;
        var vec_Shoulder_Hip = r_Shoulder - r_Hip;

        body.transform.rotation = Quaternion.Euler(Vector3.zero);
        body.transform.position = Vector3.zero;

        var forward_Body_World = body.transform.forward;
        //var forward_Body_World = body.transform.InverseTransformDirection(body.transform.forward);
        body.transform.Rotate(forward_Body_World, -beta_Body * Mathf.Rad2Deg);

        //var up_Body_World = body.transform.up;
        var up_Body_World = body.transform.InverseTransformDirection(body.transform.up);
        body.transform.Rotate(up_Body_World, -gamma_Body * Mathf.Rad2Deg);

        //var right_Body_World = body.transform.right;
        var right_Body_World = body.transform.InverseTransformDirection(body.transform.right);
        body.transform.Rotate(right_Body_World, -alpha_Body * Mathf.Rad2Deg);


        //body.transform.forward = NMC.convert_Matrix_To_Vector3(vec_R_L);
        //body.transform.up = NMC.convert_Matrix_To_Vector3(vec_Shoulder_Hip);
        //body.transform.position = NMC.convert_Matrix_To_Vector3((r_Shoulder + l_Shoulder + r_Hip + l_Hip) / 4);

        body.transform.position = new Vector3(x_Head, z_Head, y_Head) + body.transform.forward * height_Body * 0.5f;

    }

    void find_F()
    {
        var coeffs_Ddr_Arm_Bottom = NMC.FFD_Coeffs_Ddr_Arm_Bottom(dr_Beta_Hand, dr_Alpha_Hand, g, length_Hand, m_Hand, r_Alpha_Hand, r_Beta_Hand, r_Tau_Beta_Shoulder, r_Tau_Alpha_Shoulder);

        var coeffs_Ddl_Arm_Bottom = NMC.FFD_Coeffs_Ddl_Arm_Bottom(dl_Beta_Hand, dl_Alpha_Hand, g, l_Alpha_Hand, l_Beta_Hand, l_Tau_Beta_Shoulder, l_Tau_Alpha_Shoulder, length_Hand, m_Hand);

        var coeffs_Ddr_Shoulder = NMC.FFD_Coeffs_Ddr_Shoulder(alpha_Body, beta_Body, dalpha_Body, dbeta_Body, depth_Body, dgamma_Body, g, gamma_Body, height_Body, l_Alpha_Hand, l_Tau_Beta_Shoulder, l_Tau_Alpha_Shoulder, m_Body, r_Alpha_Hand, r_Tau_Beta_Shoulder, r_Tau_Alpha_Shoulder, width_Body);
        var coeffs_Ddl_Shoulder = NMC.FFD_Coeffs_Ddl_Shoulder(alpha_Body, beta_Body, dalpha_Body, dbeta_Body, depth_Body, dgamma_Body, g, gamma_Body, height_Body, l_Alpha_Hand, l_Tau_Beta_Shoulder, l_Tau_Alpha_Shoulder, m_Body, r_Alpha_Hand, r_Tau_Beta_Shoulder, r_Tau_Alpha_Shoulder, width_Body);

        var diff_Coeffs_Ddr_Arm_Botom = coeffs_Ddr_Arm_Bottom - coeffs_Ddr_Shoulder;
        var diff_Coeffs_Ddl_Arm_Botom = coeffs_Ddl_Arm_Bottom - coeffs_Ddl_Shoulder;

        var diff_Coeffs = Matrix.MatrixDockRows(diff_Coeffs_Ddr_Arm_Botom, diff_Coeffs_Ddl_Arm_Botom);

        var coeffs_Matrix = Matrix.MatrixTrim(diff_Coeffs, 0, 6, 0, 6);

        var coeffs_Target = Matrix.MatrixTrim(diff_Coeffs, 0, 6, 6, 1);

        //var coeffs_Matrix = new Matrix(
        //    new float[][]
        //    {
        //        new float[]{ diff_Coeffs_Ddl_Arm_Botom[0, 0], diff_Coeffs_Ddl_Arm_Botom[0, 1], diff_Coeffs_Ddl_Arm_Botom[0, 2], diff_Coeffs_Ddl_Arm_Botom[0, 3],
        //            diff_Coeffs_Ddl_Arm_Botom[0, 4], diff_Coeffs_Ddl_Arm_Botom[0, 5] },
        //        new float[]{ diff_Coeffs_Ddl_Arm_Botom[1, 0], diff_Coeffs_Ddl_Arm_Botom[1, 1], diff_Coeffs_Ddl_Arm_Botom[1, 2], diff_Coeffs_Ddl_Arm_Botom[1, 3],
        //            diff_Coeffs_Ddl_Arm_Botom[1, 4], diff_Coeffs_Ddl_Arm_Botom[1, 5] },
        //        new float[]{ diff_Coeffs_Ddl_Arm_Botom[2, 0], diff_Coeffs_Ddl_Arm_Botom[2, 1], diff_Coeffs_Ddl_Arm_Botom[2, 2], diff_Coeffs_Ddl_Arm_Botom[2, 3],
        //            diff_Coeffs_Ddl_Arm_Botom[2, 4], diff_Coeffs_Ddl_Arm_Botom[2, 5] },
        //        new float[]{ diff_Coeffs_Ddr_Arm_Botom[0, 0], diff_Coeffs_Ddr_Arm_Botom[0, 1], diff_Coeffs_Ddr_Arm_Botom[0, 2], diff_Coeffs_Ddr_Arm_Botom[0, 3],
        //            diff_Coeffs_Ddr_Arm_Botom[0, 4], diff_Coeffs_Ddr_Arm_Botom[0, 5] },
        //        new float[]{ diff_Coeffs_Ddr_Arm_Botom[1, 0], diff_Coeffs_Ddr_Arm_Botom[1, 1], diff_Coeffs_Ddr_Arm_Botom[1, 2], diff_Coeffs_Ddr_Arm_Botom[1, 3],
        //            diff_Coeffs_Ddr_Arm_Botom[1, 4], diff_Coeffs_Ddr_Arm_Botom[1, 5] },
        //        new float[]{ diff_Coeffs_Ddr_Arm_Botom[2, 0], diff_Coeffs_Ddr_Arm_Botom[2, 1], diff_Coeffs_Ddr_Arm_Botom[2, 2], diff_Coeffs_Ddr_Arm_Botom[2, 3],
        //            diff_Coeffs_Ddr_Arm_Botom[2, 4], diff_Coeffs_Ddr_Arm_Botom[2, 5] }
        //    });

        //var coeffs_Target = new Matrix(new float[][]
        //{
        //    new float[]{ diff_Coeffs_Ddl_Arm_Botom[0, 6] },
        //    new float[]{ diff_Coeffs_Ddl_Arm_Botom[1, 6] },
        //    new float[]{ diff_Coeffs_Ddl_Arm_Botom[2, 6] },
        //    new float[]{ diff_Coeffs_Ddr_Arm_Botom[0, 6] },
        //    new float[]{ diff_Coeffs_Ddr_Arm_Botom[1, 6] },
        //    new float[]{ diff_Coeffs_Ddr_Arm_Botom[2, 6] },
        //});


        var f_All = Matrix.MatrixInverse(coeffs_Matrix) * (-coeffs_Target);

        l_F_X = f_All[0, 0];
        l_F_Y = f_All[1, 0];
        l_F_Z = f_All[2, 0];

        r_F_X = f_All[3, 0];
        r_F_Y = f_All[4, 0];
        r_F_Z = f_All[5, 0];

        //r_F_X = f_All[0, 0];
        //r_F_Y = f_All[1, 0];
        //r_F_Z = f_All[2, 0];

        //l_F_X = f_All[3, 0];
        //l_F_Y = f_All[4, 0];
        //l_F_Z = f_All[5, 0];
    }

    void find_Restrained_Velocity()
    {

        //var dl_Arm_Bottom = NMC.find_Dl_Arm_Bottom(dl_Beta_Hand, dl_Alpha_Hand, l_Alpha_Hand, l_Beta_Hand, length_Hand);
        //var dr_Arm_Bottom = NMC.find_Dr_Arm_Bottom(dr_Beta_Hand, dr_Alpha_Hand, length_Hand, r_Alpha_Hand, r_Beta_Hand);

        //dx_Head = (dl_Arm_Bottom[0, 0] + dr_Arm_Bottom[0, 0]) / 2;
        //dy_Head = (dl_Arm_Bottom[1, 0] + dr_Arm_Bottom[1, 0]) / 2;
        //dz_Head = (dl_Arm_Bottom[2, 0] + dr_Arm_Bottom[2, 0]) / 2;

        //var restrained_Velocity = NMC.find_Restrained_Velocity(alpha_Body, beta_Body, dalpha_Body, dgamma_Body, dl_Beta_Hand, dl_Alpha_Hand, dr_Beta_Hand, dr_Alpha_Hand, gamma_Body, l_Alpha_Hand, l_Beta_Hand, length_Hand, r_Alpha_Hand, r_Beta_Hand, width_Body);

        //if (!float.IsNaN(restrained_Velocity[0]))
        //{
        //    dbeta_Body = restrained_Velocity[0];
        //}
        //if (!float.IsNaN(restrained_Velocity[1]))
        //{
        //    dx_Head = restrained_Velocity[1];
        //}
        //if (!float.IsNaN(restrained_Velocity[2]))
        //{
        //    dy_Head = restrained_Velocity[2];
        //}
        //if (!float.IsNaN(restrained_Velocity[3]))
        //{
        //    dz_Head = restrained_Velocity[3];
        //}
    }

    void find_F_Spring()
    {
        //var r_Arm_Bottom = NMC.find_R_Arm_Bottom(length_Hand, r_Alpha_Hand, r_Beta_Hand, r_X_Fixed, r_Y_Fixed, r_Z_Fixed);
        //var l_Arm_Bottom = NMC.find_L_Arm_Bottom(l_Alpha_Hand, l_Beta_Hand, l_X_Fixed, l_Y_Fixed, l_Z_Fixed, length_Hand);

        //var r_Shoulder = NMC.find_R_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);
        //var l_Shoulder = NMC.find_L_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);

        //var diff_R_Arm_Bottm_Shoulder = r_Arm_Bottom - r_Shoulder;
        //var diff_L_Arm_Bottm_Shoulder = l_Arm_Bottom - l_Shoulder;

        //var r_F = -k * diff_R_Arm_Bottm_Shoulder;
        //var l_F = -k * diff_L_Arm_Bottm_Shoulder;

        //r_F_X = r_F[0, 0];
        //r_F_Y = r_F[1, 0];
        //r_F_Z = r_F[2, 0];

        //l_F_X = l_F[0, 0];
        //l_F_Y = l_F[1, 0];
        //l_F_Z = l_F[2, 0];

    }
}
