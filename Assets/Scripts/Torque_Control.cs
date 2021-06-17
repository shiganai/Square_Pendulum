using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Torque_Control : MonoBehaviour
{
    private GameObject arm_L;
    private GameObject arm_R;
    private GameObject body;
    private GameObject r_Shoulder_Go, l_Shoulder_Go, r_Hip_Go, l_Hip_Go, r_Rib_Go, l_Rib_Go;

    private float width_Body;
    private float height_Body;
    private float depth_Body;
    private float m_Body;
    private float m_Hand;
    private float length_Hand, radius_Hand;
    public float g = 1f;

    public float alpha_Body;
    public float beta_Body;
    public float gamma_Body;
    public float x_Head;
    public float y_Head;
    public float z_Head;

    public float r_Alpha_Hand;
    public float r_Beta_Hand;
    public float r_Gamma_Hand;

    public float l_Alpha_Hand;
    public float l_Beta_Hand;
    public float l_Gamma_Hand;

    public float dalpha_Body;
    public float dbeta_Body;
    public float dgamma_Body;
    private float dx_Head;
    private float dy_Head;
    private float dz_Head;

    private float dr_Alpha_Hand;
    private float dr_Beta_Hand;
    private float dr_Gamma_Hand;

    private float dl_Alpha_Hand;
    private float dl_Beta_Hand;
    private float dl_Gamma_Hand;

    private float r_X_Fixed, r_Y_Fixed, r_Z_Fixed;
    private float l_X_Fixed, l_Y_Fixed, l_Z_Fixed;

    public float r_F_X, r_F_Y, r_F_Z;
    private float l_F_X, l_F_Y, l_F_Z;

    public float k, c;

    private matrix r_P_Fixed;
    private matrix l_P_Fixed;

    public float torque_Body_L_Max;
    public float torque_Body_R_Max;

    public float dd_Threshold = 1e-1f;
    public bool toReset = false;

    public float r_Tau_Alpha_Shoulder, l_Tau_Alpha_Shoulder;
    public float r_Tau_Beta_Shoulder, l_Tau_Beta_Shoulder;
    public float r_Tau_Gamma_Shoulder, l_Tau_Gamma_Shoulder;

    public bool is_Playing;
    public bool isHFD, isFFD;

    public float velocity_Zero;

    public drag_Control graph_Controler;
    public slider_contro slider;
    public scroll_Control scroll;


    // Start is called before the first frame update
    void Start()
    {
        arm_L = GameObject.Find("arm_L");
        arm_R = GameObject.Find("arm_R");
        body = GameObject.Find("body");
        r_Shoulder_Go = GameObject.Find("r_Shoulder");
        l_Shoulder_Go = GameObject.Find("l_Shoulder");
        r_Hip_Go = GameObject.Find("r_Hip");
        l_Hip_Go = GameObject.Find("l_Hip");
        r_Rib_Go = GameObject.Find("r_Rib");
        l_Rib_Go = GameObject.Find("l_Rib");

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
        radius_Hand = 1f;
        //g = 0f;
        //k = 1e2f;

        alpha_Body = -90f * Mathf.Deg2Rad;
        beta_Body = 0;
        gamma_Body = 0;
        x_Head = 0f;
        y_Head = 0f;
        z_Head = 0f;

        r_Alpha_Hand = -90f * Mathf.Deg2Rad;
        r_Beta_Hand = 0f * Mathf.Deg2Rad;
        r_Gamma_Hand = 0f * Mathf.Deg2Rad;

        l_Alpha_Hand = -90f * Mathf.Deg2Rad;
        l_Beta_Hand = 0f * Mathf.Deg2Rad;
        l_Gamma_Hand = 0f * Mathf.Deg2Rad;

        dalpha_Body = velocity_Zero;
        dbeta_Body = 0;
        dgamma_Body = 0;
        dx_Head = 0f;
        dy_Head = 0f;
        dz_Head = 0f;

        dr_Alpha_Hand = velocity_Zero;
        dr_Beta_Hand = 0;

        dl_Alpha_Hand = velocity_Zero;
        dl_Beta_Hand = 0;

        r_Tau_Alpha_Shoulder = 0;
        l_Tau_Alpha_Shoulder = 0;
        r_Tau_Beta_Shoulder = 0;
        l_Tau_Beta_Shoulder = 0;

        r_P_Fixed = matrix.convert_Vector3_To_Matrix(Vector3.zero);
        l_P_Fixed = matrix.convert_Vector3_To_Matrix(Vector3.zero);

        r_X_Fixed = r_P_Fixed[0, 0];
        r_Y_Fixed = r_P_Fixed[1, 0];
        r_Z_Fixed = r_P_Fixed[2, 0];

        l_X_Fixed = l_P_Fixed[0, 0];
        l_Y_Fixed = l_P_Fixed[1, 0];
        l_Z_Fixed = l_P_Fixed[2, 0];

        var r_Arm_Bottom = FRD.FRD_R_Arm_Bottom(length_Hand, r_Alpha_Hand, r_Beta_Hand, r_X_Fixed, r_Y_Fixed, r_Z_Fixed);
        var l_Arm_Bottom = FRD.FRD_L_Arm_Bottom(l_Alpha_Hand, l_Beta_Hand, l_X_Fixed, l_Y_Fixed, l_Z_Fixed, length_Hand);

        var r_Shoulder = FRD.FRD_R_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);
        var l_Shoulder = FRD.FRD_L_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);

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

        if (is_Playing)
        {
            find_F_Spring();

            calc_FFD();
            //calc_HFD();

            //if (isFFD)
            //{
            //    isHFD = false;
            //    calc_FFD();
            //}
            //if (isHFD)
            //{
            //    isFFD = false;
            //    calc_HFD();
            //}

        }

        if (toReset)
        {
            Initialize();
            toReset = false;
        }

        visualization();
    }

    void visualization()
    {
        var r_Arm_Bottom = FRD.FRD_R_Arm_Bottom(length_Hand, r_Alpha_Hand, r_Beta_Hand, r_X_Fixed, r_Y_Fixed, r_Z_Fixed);
        var l_Arm_Bottom = FRD.FRD_L_Arm_Bottom(l_Alpha_Hand, l_Beta_Hand, l_X_Fixed, l_Y_Fixed, l_Z_Fixed, length_Hand);

        var r_Arm_Bottom_From_Fixed = r_Arm_Bottom - r_P_Fixed;
        var l_Arm_Bottom_From_Fixed = l_Arm_Bottom - l_P_Fixed;

        arm_R.transform.forward = matrix.convert_Matrix_To_Vector3(r_Arm_Bottom_From_Fixed);

        arm_R.transform.position = matrix.convert_Matrix_To_Vector3(r_Arm_Bottom_From_Fixed * 0.5f + r_P_Fixed);


        arm_L.transform.forward = matrix.convert_Matrix_To_Vector3(l_Arm_Bottom_From_Fixed);

        arm_L.transform.position = matrix.convert_Matrix_To_Vector3(l_Arm_Bottom_From_Fixed * 0.5f + l_P_Fixed);


        //var head_P = (r_Arm_Bottom + l_Arm_Bottom) / 2;
        //x_Head = head_P[0, 0];
        //y_Head = head_P[1, 0];
        //z_Head = head_P[2, 0];

        var r_Shoulder = FRD.FRD_R_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);
        var l_Shoulder = FRD.FRD_L_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);

        var r_Hip = FRD.FRD_R_Hip(alpha_Body, beta_Body, gamma_Body, height_Body, width_Body, x_Head, y_Head, z_Head);
        var l_Hip = FRD.FRD_L_Hip(alpha_Body, beta_Body, gamma_Body, height_Body, width_Body, x_Head, y_Head, z_Head);

        var vec_R_L = r_Shoulder - l_Shoulder;
        var vec_Shoulder_Hip = r_Shoulder - r_Hip;

        body.transform.rotation = Quaternion.Euler(Vector3.zero);
        body.transform.position = Vector3.zero;

        var right_Body_World = Vector3.right;
        //var right_Body_World = body.transform.right;
        //var right_Body_World = body.transform.InverseTransformDirection(body.transform.right);
        body.transform.Rotate(right_Body_World, -alpha_Body * Mathf.Rad2Deg);

        var up_Body_World = Vector3.up;
        //var up_Body_World = body.transform.up;
        //var up_Body_World = body.transform.InverseTransformDirection(body.transform.up);
        body.transform.Rotate(up_Body_World, -gamma_Body * Mathf.Rad2Deg);

        var forward_Body_World = Vector3.forward;
        //var forward_Body_World = body.transform.forward;
        //var forward_Body_World = body.transform.InverseTransformDirection(body.transform.forward);
        body.transform.Rotate(forward_Body_World, -beta_Body * Mathf.Rad2Deg);

        var up_Debugg = matrix.convert_Matrix_To_Vector3(vec_Shoulder_Hip).normalized;

        //body.transform.right = matrix.convert_Matrix_To_Vector3(vec_R_L);
        //body.transform.forward = up_Debugg;
        //body.transform.position = NMC.convert_Matrix_To_Vector3((r_Shoulder + l_Shoulder + r_Hip + l_Hip) / 4);

        body.transform.position = new Vector3(x_Head, z_Head, y_Head) + body.transform.forward * height_Body * 0.5f;

        r_Shoulder_Go.transform.position = matrix.convert_Matrix_To_Vector3(r_Shoulder);
        l_Shoulder_Go.transform.position = matrix.convert_Matrix_To_Vector3(l_Shoulder);
        r_Hip_Go.transform.position = matrix.convert_Matrix_To_Vector3(r_Hip);
        l_Hip_Go.transform.position = matrix.convert_Matrix_To_Vector3(l_Hip);
        r_Rib_Go.transform.position = matrix.convert_Matrix_To_Vector3((r_Shoulder + r_Hip) / 2);
        l_Rib_Go.transform.position = matrix.convert_Matrix_To_Vector3((l_Shoulder + l_Hip) / 2);

    }

    void find_F_Spring()
    {
        var r_Arm_Bottom = FRD.FRD_R_Arm_Bottom(length_Hand, r_Alpha_Hand, r_Beta_Hand, r_X_Fixed, r_Y_Fixed, r_Z_Fixed);
        var l_Arm_Bottom = FRD.FRD_L_Arm_Bottom(l_Alpha_Hand, l_Beta_Hand, l_X_Fixed, l_Y_Fixed, l_Z_Fixed, length_Hand);

        var r_Shoulder = FRD.FRD_R_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);
        var l_Shoulder = FRD.FRD_L_Shoulder(alpha_Body, beta_Body, gamma_Body, width_Body, x_Head, y_Head, z_Head);

        var diff_R_Arm_Bottm_Shoulder = r_Arm_Bottom - r_Shoulder;
        var diff_L_Arm_Bottm_Shoulder = l_Arm_Bottom - l_Shoulder;

        var dr_Shoulder = FRD.FRD_Dr_Shoulder(alpha_Body, beta_Body, dalpha_Body, dbeta_Body, dgamma_Body, dx_Head, dy_Head, dz_Head, gamma_Body, width_Body);
        var dl_Shoulder = FRD.FRD_Dl_Shoulder(alpha_Body, beta_Body, dalpha_Body, dbeta_Body, dgamma_Body, dx_Head, dy_Head, dz_Head, gamma_Body, width_Body);

        var dr_Arm_Bottom = FRD.FRD_Dr_Arm_Bottom(dr_Beta_Hand, dr_Alpha_Hand, length_Hand, r_Alpha_Hand, r_Beta_Hand);
        var dl_Arm_Bottom = FRD.FRD_Dl_Arm_Bottom(dl_Beta_Hand, dl_Alpha_Hand, l_Alpha_Hand, l_Beta_Hand, length_Hand);

        var diff_Dr_Arm_Bottm_Shoulder = dr_Arm_Bottom - dr_Shoulder;
        var diff_Dl_Arm_Bottm_Shoulder = dl_Arm_Bottom - dl_Shoulder;

        var r_F = k * diff_R_Arm_Bottm_Shoulder + c * diff_Dr_Arm_Bottm_Shoulder;
        var l_F = k * diff_L_Arm_Bottm_Shoulder + c * diff_Dl_Arm_Bottm_Shoulder;

        r_F_X = r_F[0, 0];
        r_F_Y = r_F[1, 0];
        r_F_Z = r_F[2, 0];

        l_F_X = l_F[0, 0];
        l_F_Y = l_F[1, 0];
        l_F_Z = l_F[2, 0];

    }

    void calc_FFD()
    {
        r_Tau_Alpha_Shoulder = slider.GetValue() * torque_Body_R_Max;
        l_Tau_Alpha_Shoulder = r_Tau_Alpha_Shoulder;

        r_Tau_Alpha_Shoulder += (2 * scroll.GetValue() - 1) * torque_Body_L_Max;
        l_Tau_Alpha_Shoulder -= (2 * scroll.GetValue() - 1) * torque_Body_L_Max;

        var dds_R_Hand = FFD_Arm_R.FFD_Dds_Arm_R(dr_Beta_Hand, dr_Gamma_Hand, dr_Alpha_Hand, g, length_Hand, m_Hand, r_Alpha_Hand, r_Beta_Hand, r_F_X, r_F_Y, r_F_Z, r_Gamma_Hand, r_Tau_Beta_Shoulder, r_Tau_Gamma_Shoulder, r_Tau_Alpha_Shoulder, radius_Hand);
        var dds_L_Hand = FFD_Arm_L.FFD_Dds_Arm_L(dl_Beta_Hand, dl_Gamma_Hand, dl_Alpha_Hand, g, l_Alpha_Hand, l_Beta_Hand, l_F_X, l_F_Y, l_F_Z, l_Gamma_Hand, l_Tau_Beta_Shoulder, l_Tau_Gamma_Shoulder, l_Tau_Alpha_Shoulder, length_Hand, m_Hand, radius_Hand);
        //var dds_Body = FFD.FFD_Dds_Body(alpha_Body, beta_Body, dalpha_Body, depth_Body, dgamma_Body, g, gamma_Body, height_Body, l_Alpha_Hand, l_F_X, l_F_Y, l_F_Z, l_Tau_Beta_Shoulder, l_Tau_Alpha_Shoulder, m_Body, r_Alpha_Hand, r_F_X, r_F_Y, r_F_Z, r_Tau_Beta_Shoulder, r_Tau_Alpha_Shoulder, width_Body);

        //var r_Tau_Alpha_Shoulder_Tmp = -r_Tau_Alpha_Shoulder;
        //var l_Tau_Alpha_Shoulder_Tmp = -l_Tau_Alpha_Shoulder;
        var dds_Body = FFD_Body.FFD_Dds_Body(alpha_Body, beta_Body, dalpha_Body, dbeta_Body, depth_Body, dgamma_Body, g, gamma_Body, height_Body, l_Alpha_Hand, l_Beta_Hand, l_F_X, l_F_Y, l_F_Z, l_Tau_Beta_Shoulder, l_Tau_Gamma_Shoulder, l_Tau_Alpha_Shoulder, m_Body, r_Alpha_Hand, r_Beta_Hand, r_F_X, r_F_Y, r_F_Z, r_Tau_Beta_Shoulder, r_Tau_Gamma_Shoulder, r_Tau_Alpha_Shoulder, width_Body);

        var ddalpha_Body = dds_Body[0];
        var ddbeta_Body = dds_Body[1];
        var ddgamma_Body = dds_Body[2];
        var ddx_Head = dds_Body[3];
        var ddy_Head = dds_Body[4];
        var ddz_Head = dds_Body[5];

        var ddr_Alpha_Hand = dds_R_Hand[0];
        var ddr_Beta_Hand = dds_R_Hand[1];

        var ddl_Alpha_Hand = dds_L_Hand[0];
        var ddl_Beta_Hand = dds_L_Hand[1];

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

        //dr_Alpha_Hand += ddr_Alpha_Hand * Time.fixedDeltaTime;
        //dr_Beta_Hand += ddr_Beta_Hand * Time.fixedDeltaTime;
        //dl_Alpha_Hand += ddl_Alpha_Hand * Time.fixedDeltaTime;
        //dl_Beta_Hand += ddl_Beta_Hand * Time.fixedDeltaTime;

        //dalpha_Body += ddalpha_Body * Time.fixedDeltaTime;
        //dbeta_Body += ddbeta_Body * Time.fixedDeltaTime;
        //dgamma_Body += ddgamma_Body * Time.fixedDeltaTime;
        //dx_Head += ddx_Head * Time.fixedDeltaTime;
        //dy_Head += ddy_Head * Time.fixedDeltaTime;
        //dz_Head += ddz_Head * Time.fixedDeltaTime;
    }

    void calc_HFD()
    {
        var ddbeta_Body = 0.1f;
        var ddgamma_Body = 0f;

        var dds_Body = HFD.HFD_Dds_Body(alpha_Body, beta_Body, dalpha_Body, dbeta_Body, ddbeta_Body, ddgamma_Body, depth_Body, dgamma_Body, g, gamma_Body, height_Body, l_Alpha_Hand, l_Beta_Hand, l_F_X, l_F_Y, l_F_Z, l_Tau_Alpha_Shoulder, m_Body, r_Alpha_Hand, r_Beta_Hand, r_F_X, r_F_Y, r_F_Z, r_Tau_Alpha_Shoulder, width_Body);

        var ddalpha_Body = dds_Body[0];
        var ddx_Head = dds_Body[1];
        var ddy_Head = dds_Body[2];
        var ddz_Head = dds_Body[3];

        r_Tau_Alpha_Shoulder = 0;
        r_Tau_Beta_Shoulder = dds_Body[4];
        r_Tau_Gamma_Shoulder = dds_Body[5];

        var dds_R_Hand = FFD_Arm_R.FFD_Dds_Arm_R(dr_Beta_Hand, dr_Gamma_Hand, dr_Alpha_Hand, g, length_Hand, m_Hand, r_Alpha_Hand, r_Beta_Hand, r_F_X, r_F_Y, r_F_Z, r_Gamma_Hand, r_Tau_Beta_Shoulder, r_Tau_Gamma_Shoulder, r_Tau_Alpha_Shoulder, radius_Hand);
        var dds_L_Hand = FFD_Arm_L.FFD_Dds_Arm_L(dl_Beta_Hand, dl_Gamma_Hand, dl_Alpha_Hand, g, l_Alpha_Hand, l_Beta_Hand, l_F_X, l_F_Y, l_F_Z, l_Gamma_Hand, l_Tau_Beta_Shoulder, l_Tau_Gamma_Shoulder, l_Tau_Alpha_Shoulder, length_Hand, m_Hand, radius_Hand);

        var ddr_Alpha_Hand = dds_R_Hand[0];
        var ddr_Beta_Hand = dds_R_Hand[1];

        var ddl_Alpha_Hand = dds_L_Hand[0];
        var ddl_Beta_Hand = dds_L_Hand[1];

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


        dr_Alpha_Hand += ddr_Alpha_Hand * Time.fixedDeltaTime;
        dr_Beta_Hand += ddr_Beta_Hand * Time.fixedDeltaTime;
        dl_Alpha_Hand += ddl_Alpha_Hand * Time.fixedDeltaTime;
        dl_Beta_Hand += ddl_Beta_Hand * Time.fixedDeltaTime;

        dalpha_Body += ddalpha_Body * Time.fixedDeltaTime;
        dbeta_Body += ddbeta_Body * Time.fixedDeltaTime;
        dgamma_Body += ddgamma_Body * Time.fixedDeltaTime;
        dx_Head += ddx_Head * Time.fixedDeltaTime;
        dy_Head += ddy_Head * Time.fixedDeltaTime;
        dz_Head += ddz_Head * Time.fixedDeltaTime;
    }
}
