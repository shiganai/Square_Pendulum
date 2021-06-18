using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HFD_Alpha
{
    public static float[] HFD_Dds_Body_Alpha(float alpha_Body,float dalpha_Body,float ddalpha_Body,float depth_Body,float g,float height_Body,float l_Beta_Hand,float l_F_X,float l_F_Y,float l_F_Z,float l_Tau_Gamma_Shoulder,float m_Body,float r_Beta_Hand,float r_F_X,float r_F_Y,float r_F_Z,float r_Tau_Gamma_Shoulder)
    {

float t2 = Mathf.Cos(alpha_Body);
float t3 = Mathf.Sin(alpha_Body);
float t4 = Mathf.Sin(l_Beta_Hand);
float t5 = Mathf.Sin(r_Beta_Hand);
float t6 = g *m_Body;
float t7 = Mathf.Pow(dalpha_Body,2f);
float t8 = Mathf.Pow(depth_Body,2f);
float t9 = Mathf.Pow(height_Body,2f);
float t14 = 1.0f /m_Body;
float ddx_Head = t14 *(l_F_X+r_F_X);
    float t10 = Mathf.Pow(t2,2f);
    float t12 = Mathf.Pow(t3,2f);
    float t15 = -t6;
    float t16 = ddalpha_Body *height_Body *t3 *2.0f;
    float t17 = height_Body *t2 *t7 *2.0f;
    float t18 = (ddalpha_Body *height_Body *m_Body *t2) /2.0f;
    float t20 = (height_Body *m_Body *t3 *t7) /2.0f;
    float t11 = Mathf.Pow(t10,2f);
    float t13 = Mathf.Pow(t12,2f);
    float t19 = -t18;
    float t21 = t16+t17;
    float t22 = (m_Body *t21) /4.0f;
    float t24 = l_F_Z+r_F_Z+t15+t19+t20;
    float t23 = l_F_Y+r_F_Y+t22;
    float ddy_Head = t14 *t23;
    float ddz_Head = t14 *t24;
    float r_Tau_Alpha_Shoulder = (l_Tau_Gamma_Shoulder *t4 *t10+l_Tau_Gamma_Shoulder *t4 *t12+r_Tau_Gamma_Shoulder *t5 *t10+r_Tau_Gamma_Shoulder *t5 *t12) /(t10 *2.0f+t12 *2.0f)+(height_Body *t2 *t6) /4.0f+(height_Body *t2 *t24) /4.0f-(height_Body *t3 *t23) /4.0f+(ddalpha_Body *m_Body *t8 *t11) /2.4e+1f+(ddalpha_Body *m_Body *t9 *t10) /8.0f+(ddalpha_Body *m_Body *t9 *t11) /2.4e+1f+(ddalpha_Body *m_Body *t8 *t13) /2.4e+1f+(ddalpha_Body *m_Body *t9 *t12) /8.0f+(ddalpha_Body *m_Body *t9 *t13) /2.4e+1f+(ddalpha_Body *m_Body *t8 *t10 *t12) /1.2e+1f+(ddalpha_Body *m_Body *t9 *t10 *t12) /1.2e+1f;

        return new float[] { ddx_Head,ddy_Head,ddz_Head,r_Tau_Alpha_Shoulder };
    }
}
