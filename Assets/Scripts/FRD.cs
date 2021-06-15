using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FRD
{
    public static matrix FRD_Ddl_Arm_Bottom(float ddl_Beta_Hand, float ddl_Alpha_Hand, float dl_Beta_Hand, float dl_Alpha_Hand, float l_Alpha_Hand, float l_Beta_Hand, float length_Hand)
    {

        float t2 = Mathf.Cos(l_Alpha_Hand);
        float t3 = Mathf.Cos(l_Beta_Hand);
        float t4 = Mathf.Sin(l_Alpha_Hand);
        float t5 = Mathf.Sin(l_Beta_Hand);
        float t6 = Mathf.Pow(dl_Beta_Hand, 2f);
        float t7 = Mathf.Pow(dl_Alpha_Hand, 2f);
        return matrix.matrixTranspose(new matrix(new float[,]
        {
            {ddl_Beta_Hand * length_Hand * t3 - length_Hand * t5 * t6, -ddl_Beta_Hand * length_Hand * t2 * t5 - ddl_Alpha_Hand * length_Hand * t3 * t4 - length_Hand * t2 * t3 * t6 - length_Hand * t2 * t3 * t7 + dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t4 * t5 * 2.0f, -ddl_Beta_Hand * length_Hand * t4 * t5 + ddl_Alpha_Hand * length_Hand * t2 * t3 - length_Hand * t3 * t4 * t6 - length_Hand * t3 * t4 * t7 - dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t2 * t5 * 2.0f }
        }));

    }

    public static matrix FRD_L_Arm_Bottom(float l_Alpha_Hand, float l_Beta_Hand, float l_X_Fixed, float l_Y_Fixed, float l_Z_Fixed, float length_Hand)
    {

        float t2 = Mathf.Cos(l_Beta_Hand);
        return matrix.matrixTranspose(new matrix(new float[,]
        {
            {l_X_Fixed + length_Hand * Mathf.Sin(l_Beta_Hand), l_Y_Fixed + length_Hand * t2 * Mathf.Cos(l_Alpha_Hand), l_Z_Fixed + length_Hand * t2 * Mathf.Sin(l_Alpha_Hand)}
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
        return matrix.matrixTranspose(new matrix(new float[,]
        {
            {x_Head - height_Body * t7 - (t3 * t4 * width_Body) / 2.0f, y_Head - (width_Body * (t5 * t6 + t2 * t3 * t7)) / 2.0f + height_Body * t2 * t4, z_Head + (width_Body * (t2 * t6 - t3 * t5 * t7)) / 2.0f + height_Body * t4 * t5}
        }));

    }
    public static matrix FRD_L_Shoulder(float alpha_Body, float beta_Body, float gamma_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Sin(alpha_Body);
        float t5 = Mathf.Sin(beta_Body);
        float t6 = Mathf.Sin(gamma_Body);
        return matrix.matrixTranspose(new matrix(new float[,]
        {
            {x_Head - (t3 * width_Body * Mathf.Cos(gamma_Body)) / 2.0f, y_Head - (width_Body * (t4 * t5 + t2 * t3 * t6)) / 2.0f, z_Head + (width_Body * (t2 * t5 - t3 * t4 * t6)) / 2.0f}
        }));

    }

    public static matrix FRD_R_Arm_Bottom(float length_Hand, float r_Alpha_Hand, float r_Beta_Hand, float r_X_Fixed, float r_Y_Fixed, float r_Z_Fixed)
    {

        float t2 = Mathf.Cos(r_Beta_Hand);
        return matrix.matrixTranspose(new matrix(new float[,]
        {
            {r_X_Fixed - length_Hand * Mathf.Sin(r_Beta_Hand), r_Y_Fixed + length_Hand * t2 * Mathf.Cos(r_Alpha_Hand), r_Z_Fixed + length_Hand * t2 * Mathf.Sin(r_Alpha_Hand) }
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
        return matrix.matrixTranspose(new matrix(new float[,]
        {
            {x_Head - height_Body * t7 + (t3 * t4 * width_Body) / 2.0f, y_Head + (width_Body * (t5 * t6 + t2 * t3 * t7)) / 2.0f + height_Body * t2 * t4, z_Head - (width_Body * (t2 * t6 - t3 * t5 * t7)) / 2.0f + height_Body * t4 * t5}
        }));

    }
    public static matrix FRD_R_Shoulder(float alpha_Body, float beta_Body, float gamma_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Sin(alpha_Body);
        float t5 = Mathf.Sin(beta_Body);
        float t6 = Mathf.Sin(gamma_Body);
        return matrix.matrixTranspose(new matrix(new float[,]
        {
            {x_Head + (t3 * width_Body * Mathf.Cos(gamma_Body)) / 2.0f, y_Head + (width_Body * (t4 * t5 + t2 * t3 * t6)) / 2.0f, z_Head - (width_Body * (t2 * t5 - t3 * t4 * t6)) / 2.0f }
        }));
    }

}
