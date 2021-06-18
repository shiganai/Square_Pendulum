using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FRD
{

    public static matrix FRD_R_Arm_Bottom(float length_Hand, float r_Alpha_Hand, float r_Beta_Hand, float r_X_Fixed, float r_Y_Fixed, float r_Z_Fixed)
    {

        float t2 = Mathf.Cos(r_Beta_Hand);
        return matrix.transpose(new matrix(new float[,]
        {
            {r_X_Fixed - length_Hand * Mathf.Sin(r_Beta_Hand), r_Y_Fixed + length_Hand * t2 * Mathf.Cos(r_Alpha_Hand), r_Z_Fixed + length_Hand * t2 * Mathf.Sin(r_Alpha_Hand) }
        }));
    }

    public static matrix FRD_L_Arm_Bottom(float l_Alpha_Hand, float l_Beta_Hand, float l_X_Fixed, float l_Y_Fixed, float l_Z_Fixed, float length_Hand)
    {

        float t2 = Mathf.Cos(l_Beta_Hand);
        return matrix.transpose(new matrix(new float[,]
        {
            {l_X_Fixed + length_Hand * Mathf.Sin(l_Beta_Hand), l_Y_Fixed + length_Hand * t2 * Mathf.Cos(l_Alpha_Hand), l_Z_Fixed + length_Hand * t2 * Mathf.Sin(l_Alpha_Hand)}
        }));

    }

    public static matrix FRD_R_Shoulder(float alpha_Body, float beta_Body, float gamma_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Sin(alpha_Body);
        float t5 = Mathf.Sin(beta_Body);
        float t6 = Mathf.Sin(gamma_Body);
        return matrix.transpose(new matrix(new float[,]
        {
            {x_Head + (t3 * width_Body * Mathf.Cos(gamma_Body)) / 2.0f, y_Head + (width_Body * (t4 * t5 + t2 * t3 * t6)) / 2.0f, z_Head - (width_Body * (t2 * t5 - t3 * t4 * t6)) / 2.0f }
        }));
    }

    public static matrix FRD_L_Shoulder(float alpha_Body, float beta_Body, float gamma_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Sin(alpha_Body);
        float t5 = Mathf.Sin(beta_Body);
        float t6 = Mathf.Sin(gamma_Body);
        return matrix.transpose(new matrix(new float[,]
        {
            {x_Head - (t3 * width_Body * Mathf.Cos(gamma_Body)) / 2.0f, y_Head - (width_Body * (t4 * t5 + t2 * t3 * t6)) / 2.0f, z_Head + (width_Body * (t2 * t5 - t3 * t4 * t6)) / 2.0f}
        }));

    }

    public static matrix FRD_R_Hip(float alpha_Body, float beta_Body, float gamma_Body, float height_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        return matrix.transpose(new matrix(new float[,]
        {
            {x_Head - height_Body * t7 + (t3 * t4 * width_Body) / 2.0f, y_Head + (width_Body * (t5 * t6 + t2 * t3 * t7)) / 2.0f + height_Body * t2 * t4, z_Head - (width_Body * (t2 * t6 - t3 * t5 * t7)) / 2.0f + height_Body * t4 * t5}
        }));

    }

    public static matrix FRD_L_Hip(float alpha_Body, float beta_Body, float gamma_Body, float height_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        return matrix.transpose(new matrix(new float[,]
        {
            {x_Head - height_Body * t7 - (t3 * t4 * width_Body) / 2.0f, y_Head - (width_Body * (t5 * t6 + t2 * t3 * t7)) / 2.0f + height_Body * t2 * t4, z_Head + (width_Body * (t2 * t6 - t3 * t5 * t7)) / 2.0f + height_Body * t4 * t5}
        }));

    }

    public static matrix FRD_Dr_Arm_Bottom(float dr_Beta_Hand, float dr_Alpha_Hand, float length_Hand, float r_Alpha_Hand, float r_Beta_Hand)
    {

        float t2 = Mathf.Cos(r_Alpha_Hand);
        float t3 = Mathf.Cos(r_Beta_Hand);
        float t4 = Mathf.Sin(r_Alpha_Hand);
        float t5 = Mathf.Sin(r_Beta_Hand);
        return matrix.transpose(new matrix(new float[,]
        {
            {-dr_Beta_Hand * length_Hand * t3, -dr_Beta_Hand * length_Hand * t2 * t5 - dr_Alpha_Hand * length_Hand * t3 * t4, -dr_Beta_Hand * length_Hand * t4 * t5 + dr_Alpha_Hand * length_Hand * t2 * t3 }
        }));
    }

    public static matrix FRD_Dl_Arm_Bottom(float dl_Beta_Hand, float dl_Alpha_Hand, float l_Alpha_Hand, float l_Beta_Hand, float length_Hand)
    {

        float t2 = Mathf.Cos(l_Alpha_Hand);
        float t3 = Mathf.Cos(l_Beta_Hand);
        float t4 = Mathf.Sin(l_Alpha_Hand);
        float t5 = Mathf.Sin(l_Beta_Hand);
        return matrix.transpose(new matrix(new float[,]
        {
            {dl_Beta_Hand * length_Hand * t3, -dl_Beta_Hand * length_Hand * t2 * t5 - dl_Alpha_Hand * length_Hand * t3 * t4, -dl_Beta_Hand * length_Hand * t4 * t5 + dl_Alpha_Hand * length_Hand * t2 * t3 }
        }));

    }

    public static matrix FRD_Dr_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float dgamma_Body, float dx_Head, float dy_Head, float dz_Head, float gamma_Body, float width_Body)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        return matrix.transpose(new matrix(new float[,]
        {
            {dx_Head - (dbeta_Body * t4 * t6 * width_Body) / 2.0f - (dgamma_Body * t3 * t7 * width_Body) / 2.0f, dy_Head + (width_Body * (dalpha_Body * t2 * t6 + dbeta_Body * t3 * t5 - dalpha_Body * t3 * t5 * t7 - dbeta_Body * t2 * t6 * t7 + dgamma_Body * t2 * t3 * t4)) / 2.0f, dz_Head + (width_Body * (dalpha_Body * t5 * t6 - dbeta_Body * t2 * t3 + dalpha_Body * t2 * t3 * t7 - dbeta_Body * t5 * t6 * t7 + dgamma_Body * t3 * t4 * t5)) / 2.0f }
        }));

    }

    public static matrix FRD_Dl_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float dgamma_Body, float dx_Head, float dy_Head, float dz_Head, float gamma_Body, float width_Body)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        return matrix.transpose(new matrix(new float[,]
        {
            {dx_Head + (dbeta_Body * t4 * t6 * width_Body) / 2.0f + (dgamma_Body * t3 * t7 * width_Body) / 2.0f, dy_Head - (width_Body * (dalpha_Body * t2 * t6 + dbeta_Body * t3 * t5 - dalpha_Body * t3 * t5 * t7 - dbeta_Body * t2 * t6 * t7 + dgamma_Body * t2 * t3 * t4)) / 2.0f, dz_Head - (width_Body * (dalpha_Body * t5 * t6 - dbeta_Body * t2 * t3 + dalpha_Body * t2 * t3 * t7 - dbeta_Body * t5 * t6 * t7 + dgamma_Body * t3 * t4 * t5)) / 2.0f }
        }));

    }

    public static float FRD_Tau_Alpha_Straight(float ddbeta_Body, float dbeta_Body, float dth_Wrist, float g, float l_Body, float l_Leg, float l_Wrist_Bar, float m_Body, float m_Leg, float beta_Body, float th_Wrist)
    {
        float t2 = Mathf.Cos(beta_Body);
        float t3 = Mathf.Sin(beta_Body);
        float t4 = Mathf.Sin(th_Wrist);
        float t5 = beta_Body + th_Wrist;
        float t6 = Mathf.Pow(dbeta_Body, 2f);
        float t7 = Mathf.Pow(dth_Wrist, 2f);
        float t8 = Mathf.Pow(l_Body, 2f);
        float t9 = Mathf.Pow(l_Leg, 2f);
        float t10 = Mathf.Pow(l_Wrist_Bar, 2f);
        float t11 = Mathf.Sin(t5);
        return (l_Leg * m_Leg * (ddbeta_Body * l_Leg * 4.0f - g * t11 * 6.0f + l_Body * t3 * t7 * 6.0f - l_Wrist_Bar * t3 * t7 * 3.0f)) / 1.2e+1f - (l_Leg * l_Wrist_Bar * m_Leg * t3 * t7) / 4.0f - (l_Leg * m_Leg * (l_Leg * 2.0f + l_Body * t2 * 3.0f - l_Wrist_Bar * t2 * 3.0f) * (ddbeta_Body * m_Leg * t9 * 2.0f - g * l_Body * m_Body * t4 * 3.0f - g * l_Body * m_Leg * t4 * 6.0f - g * l_Leg * m_Leg * t11 * 3.0f + g * l_Wrist_Bar * m_Body * t4 * 6.0f + g * l_Wrist_Bar * m_Leg * t4 * 6.0f + ddbeta_Body * l_Body * l_Leg * m_Leg * t2 * 3.0f - ddbeta_Body * l_Leg * l_Wrist_Bar * m_Leg * t2 * 3.0f - l_Body * l_Leg * m_Leg * t3 * t6 * 3.0f + l_Leg * l_Wrist_Bar * m_Leg * t3 * t6 * 3.0f - dbeta_Body * dth_Wrist * l_Body * l_Leg * m_Leg * t3 * 6.0f + dbeta_Body * dth_Wrist * l_Leg * l_Wrist_Bar * m_Leg * t3 * 6.0f)) / (m_Body * t8 * 1.2e+1f + m_Body * t10 * 3.6e+1f + m_Leg * t8 * 3.6e+1f + m_Leg * t9 * 1.2e+1f + m_Leg * t10 * 3.6e+1f - l_Body * l_Wrist_Bar * m_Body * 3.6e+1f - l_Body * l_Wrist_Bar * m_Leg * 7.2e+1f + l_Body * l_Leg * m_Leg * t2 * 3.6e+1f - l_Leg * l_Wrist_Bar * m_Leg * t2 * 3.6e+1f);
    }
}
