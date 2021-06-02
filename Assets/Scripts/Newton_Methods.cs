using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Newton_Methods : MonoBehaviour
{

    public static float[] find_Dds_Body(float alpha_Body, float beta_Body, float dalpha_Body, float depth_Body, float dgamma_Body,
        float g, float gamma_Body, float height_Body, float l_F_X, float l_F_Y, float l_F_Z, float m_Body, float r_F_X, float r_F_Y, float r_F_Z, float width_Body)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        float t8 = g * m_Body;
        float t9 = Mathf.Pow(dalpha_Body, 2f);
        float t10 = Mathf.Pow(depth_Body, 2f);
        float t11 = Mathf.Pow(dgamma_Body, 2f);
        float t12 = Mathf.Pow(height_Body, 2f);
        float t14 = Mathf.Pow(width_Body, 2f);
        float t19 = 1.0f / m_Body;
        float t13 = Mathf.Pow(t12, 2f);
        float t15 = Mathf.Pow(t2, 2f);
        float t16 = Mathf.Pow(t4, 2f);
        float t17 = Mathf.Pow(t5, 2f);
        float t18 = Mathf.Pow(t7, 2f);
        float t20 = t5 * t6;
        float t21 = -t8;
        float t22 = t2 * t3;
        float t23 = t2 * t6;
        float t24 = t3 * t5;
        float t29 = t10 * t12;
        float t30 = t10 + t12;
        float t31 = t10 * t14;
        float t32 = t12 * t14;
        float t33 = t12 + t14;
        float t34 = dalpha_Body * dgamma_Body * height_Body * t5 * t7 * 4.0f;
        float t35 = dalpha_Body * dgamma_Body * height_Body * m_Body * t2 * t7;
        float t38 = height_Body * t2 * t4 * t9 * 2.0f;
        float t39 = height_Body * t2 * t4 * t11 * 2.0f;
        float t43 = (l_F_X * t3 * t7 * width_Body) / 2.0f;
        float t44 = (r_F_X * t3 * t7 * width_Body) / 2.0f;
        float t45 = (height_Body * t2 * t4 * t8) / 2.0f;
        float t46 = (height_Body * t5 * t7 * t8) / 2.0f;
        float t47 = (height_Body * m_Body * t7 * t11) / 2.0f;
        float t60 = (height_Body * m_Body * t4 * t5 * t9) / 2.0f;
        float t61 = (height_Body * m_Body * t4 * t5 * t11) / 2.0f;
        float t70 = (m_Body * t4 * t7 * t11 * t12) / 4.0f;
        float t25 = t7 * t22;
        float t26 = t7 * t23;
        float t27 = t7 * t24;
        float t28 = t7 * t20;
        float t40 = -t34;
        float t41 = 1.0f / t30;
        float t42 = 1.0f / t33;
        float t48 = -t43;
        float t49 = -t45;
        float t50 = -t46;
        float t51 = (l_F_Y * t4 * t22 * width_Body) / 2.0f;
        float t52 = (r_F_Y * t4 * t22 * width_Body) / 2.0f;
        float t53 = (l_F_Z * t4 * t24 * width_Body) / 2.0f;
        float t54 = (r_F_Z * t4 * t24 * width_Body) / 2.0f;
        float t55 = -t47;
        float t56 = t2 * t5 * t13 * t16;
        float t57 = t2 * t5 * t13 * t18;
        float t68 = t2 * t5 * t18 * t29;
        float t69 = t2 * t5 * t16 * t32;
        float t71 = -t70;
        float t73 = (dalpha_Body * dgamma_Body * m_Body * t4 * t7 * t12 * t15) / 2.0f;
        float t74 = (dalpha_Body * dgamma_Body * m_Body * t4 * t7 * t12 * t17) / 2.0f;
        float t75 = (m_Body * t4 * t7 * t9 * t12 * t15) / 4.0f;
        float t76 = t15 * t70;
        float t77 = (m_Body * t4 * t7 * t9 * t12 * t17) / 4.0f;
        float t78 = t17 * t70;
        float t85 = t13 + t29 + t31 + t32;
        float t90 = l_F_Z + r_F_Z + t21 + t35 + t60 + t61;
        float t36 = -t26;
        float t37 = -t27;
        float t58 = t20 + t25;
        float t59 = t22 + t28;
        float t62 = -t52;
        float t63 = -t54;
        float t64 = -t57;
        float t65 = l_F_X + r_F_X + t55;
        float t72 = -t68;
        float t86 = 1.0f / t85;
        float t87 = t38 + t39 + t40;
        float t66 = t23 + t37;
        float t67 = t24 + t36;
        float t79 = (l_F_Z * t58 * width_Body) / 2.0f;
        float t80 = (r_F_Z * t58 * width_Body) / 2.0f;
        float t88 = (m_Body * t87) / 4.0f;
        float t91 = t56 + t64 + t69 + t72;
        float t93 = t44 + t48 + t50 + t51 + t53 + t62 + t63 + t71 + t75 + t76 + t77 + t78;
        float t81 = -t79;
        float t82 = (l_F_Y * t66 * width_Body) / 2.0f;
        float t83 = (r_F_Y * t66 * width_Body) / 2.0f;
        float t89 = l_F_Y + r_F_Y + t88;
        float t84 = -t82;
        float t92 = t49 + t73 + t74 + t80 + t81 + t83 + t84;

        float ddalpha_Body = t19 * t41 * t92 * 1.2e+1f - height_Body * t2 * t4 * t19 * t41 * t90 * 6.0f + height_Body * t4 * t5 * t19 * t41 * t89 * 6.0f;
        float ddbeta_Body = (((l_F_Z * t59 * width_Body) / 2.0f - (l_F_Y * t67 * width_Body) / 2.0f - (r_F_Z * t59 * width_Body) / 2.0f + (r_F_Y * t67 * width_Body) / 2.0f + (l_F_X * t4 * t6 * width_Body) / 2.0f - (r_F_X * t4 * t6 * width_Body) / 2.0f) * 1.2e+1f) / (m_Body * t10 + m_Body * t14);
        float ddgamma_Body = t19 * t42 * t93 * -1.2e+1f + height_Body * t4 * t19 * t42 * t65 * 6.0f + height_Body * t2 * t7 * t19 * t42 * t89 * 6.0f + height_Body * t5 * t7 * t19 * t42 * t90 * 6.0f;
        float ddx_Head = t19 * t42 * t65 * (t33 + t12 * t16 * 3.0f) - height_Body * t4 * t19 * t42 * t93 * 6.0f + t2 * t4 * t7 * t12 * t19 * t42 * t89 * 3.0f + t4 * t5 * t7 * t12 * t19 * t42 * t90 * 3.0f;
        float ddy_Head = t19 * t86 * t90 * t91 * -3.0f + t19 * t86 * t89 * (t85 + t13 * t15 * t18 * 3.0f + t13 * t16 * t17 * 3.0f + t15 * t18 * t29 * 3.0f + t16 * t17 * t32 * 3.0f) + height_Body * t4 * t5 * t19 * t41 * t92 * 6.0f - height_Body * t2 * t7 * t19 * t42 * t93 * 6.0f + t2 * t4 * t7 * t12 * t19 * t42 * t65 * 3.0f;
        float ddz_Head = t19 * t86 * t89 * t91 * -3.0f + t19 * t86 * t90 * (t85 + t13 * t15 * t16 * 3.0f + t13 * t17 * t18 * 3.0f + t15 * t16 * t32 * 3.0f + t17 * t18 * t29 * 3.0f) - height_Body * t2 * t4 * t19 * t41 * t92 * 6.0f - height_Body * t5 * t7 * t19 * t42 * t93 * 6.0f + t4 * t5 * t7 * t12 * t19 * t42 * t65 * 3.0f;

        return new float[] { ddalpha_Body, ddbeta_Body, ddgamma_Body, ddx_Head, ddy_Head, ddz_Head };
    }

    public static float[] find_Dds_Arm_R(float dr_Beta_Hand, float dr_Alpha_Hand, float g, float length_Hand, float m_Hand, float r_Alpha_Hand, float r_Beta_Hand, float r_F_X, float r_F_Y, float r_F_Z)
    {
        float t2 = Mathf.Cos(r_Alpha_Hand);
        float t3 = Mathf.Cos(r_Beta_Hand);
        float t4 = Mathf.Sin(r_Alpha_Hand);
        float t5 = Mathf.Sin(r_Beta_Hand);
        float t6 = Mathf.Pow(dr_Beta_Hand, 2f);
        float t7 = Mathf.Pow(dr_Alpha_Hand, 2f);
        float t8 = Mathf.Pow(length_Hand, 2f);
        float t9 = Mathf.Pow(t2, 2f);
        float t10 = Mathf.Pow(t3, 2f);
        float t11 = Mathf.Pow(t4, 2f);
        float t12 = Mathf.Pow(t5, 2f);
        float ddr_Alpha_Hand = ((-length_Hand * r_F_Z * t2 + (g * length_Hand * m_Hand * t2) / 2.0f + length_Hand * r_F_Y * t3 * t4 - length_Hand * r_F_X * t4 * t5 - (m_Hand * t2 * t4 * t7 * t8) / 4.0f + (m_Hand * t2 * t4 * t6 * t8 * t10) / 4.0f + (m_Hand * t2 * t4 * t7 * t8 * t10) / 4.0f + (m_Hand * t2 * t4 * t6 * t8 * t12) / 4.0f + (m_Hand * t2 * t4 * t7 * t8 * t12) / 4.0f) * -1.2e+1f) / (m_Hand * t8 + m_Hand * t8 * t9 * 3.0f + m_Hand * t8 * t10 * t11 * 3.0f + m_Hand * t8 * t11 * t12 * 3.0f);
        float ddr_Beta_Hand = ((length_Hand * r_F_X * t2 * t3 + length_Hand * r_F_Y * t2 * t5 - (dr_Beta_Hand * dr_Alpha_Hand * m_Hand * t2 * t4 * t8 * t10) / 2.0f - (dr_Beta_Hand * dr_Alpha_Hand * m_Hand * t2 * t4 * t8 * t12) / 2.0f) * -1.2e+1f) / (m_Hand * t8 * (t9 * t10 * 3.0f + t9 * t12 * 3.0f + 1.0f));

        return new float[] { ddr_Alpha_Hand, ddr_Beta_Hand };
    }

    public static float[] find_Dds_Arm_L(float dl_Beta_Hand, float dl_Alpha_Hand, float g, float l_Alpha_Hand, float l_Beta_Hand, float l_F_X, float l_F_Y, float l_F_Z, float length_Hand, float m_Hand)
    {
        float t2 = Mathf.Cos(l_Alpha_Hand);
        float t3 = Mathf.Cos(l_Beta_Hand);
        float t4 = Mathf.Sin(l_Alpha_Hand);
        float t5 = Mathf.Sin(l_Beta_Hand);
        float t6 = Mathf.Pow(dl_Beta_Hand, 2f);
        float t7 = Mathf.Pow(dl_Alpha_Hand, 2f);
        float t8 = Mathf.Pow(length_Hand, 2f);
        float t9 = Mathf.Pow(t2, 2f);
        float t10 = Mathf.Pow(t3, 2f);
        float t11 = Mathf.Pow(t4, 2f);
        float t12 = Mathf.Pow(t5, 2f);
        float ddl_Alpha_Hand = ((-l_F_Z * length_Hand * t2 + (g * length_Hand * m_Hand * t2) / 2.0f + l_F_Y * length_Hand * t3 * t4 + l_F_X * length_Hand * t4 * t5 - (m_Hand * t2 * t4 * t7 * t8) / 4.0f + (m_Hand * t2 * t4 * t6 * t8 * t10) / 4.0f + (m_Hand * t2 * t4 * t7 * t8 * t10) / 4.0f + (m_Hand * t2 * t4 * t6 * t8 * t12) / 4.0f + (m_Hand * t2 * t4 * t7 * t8 * t12) / 4.0f) * -1.2e+1f) / (m_Hand * t8 + m_Hand * t8 * t9 * 3.0f + m_Hand * t8 * t10 * t11 * 3.0f + m_Hand * t8 * t11 * t12 * 3.0f);
        float ddl_Beta_Hand = ((l_F_X * length_Hand * t2 * t3 - l_F_Y * length_Hand * t2 * t5 + (dl_Beta_Hand * dl_Alpha_Hand * m_Hand * t2 * t4 * t8 * t10) / 2.0f + (dl_Beta_Hand * dl_Alpha_Hand * m_Hand * t2 * t4 * t8 * t12) / 2.0f) * 1.2e+1f) / (m_Hand * t8 * (t9 * t10 * 3.0f + t9 * t12 * 3.0f + 1.0f));

        return new float[] { ddl_Alpha_Hand, ddl_Beta_Hand };
    }

    public static float[][] find_Coeffs_Ddl_Arm_Bottom(float dl_Beta_Hand, float dl_Alpha_Hand, float g, float l_Alpha_Hand, float l_Beta_Hand, float length_Hand, float m_Hand)
    {

        float t2 = Mathf.Cos(l_Alpha_Hand);
        float t3 = Mathf.Cos(l_Beta_Hand);
        float t4 = Mathf.Sin(l_Alpha_Hand);
        float t5 = Mathf.Sin(l_Beta_Hand);
        float t6 = Mathf.Pow(dl_Beta_Hand, 2f);
        float t7 = Mathf.Pow(dl_Alpha_Hand, 2f);
        float t8 = Mathf.Pow(length_Hand, 2f);
        float t13 = 1.0f / length_Hand;
        float t14 = 1.0f / m_Hand;
        float t9 = Mathf.Pow(t2, 2f);
        float t10 = Mathf.Pow(t3, 2f);
        float t11 = Mathf.Pow(t4, 2f);
        float t12 = Mathf.Pow(t5, 2f);
        float t15 = m_Hand * t8;
        float t16 = (g * length_Hand * m_Hand * t2) / 2.0f;
        float t17 = t9 * t10 * 3.0f;
        float t18 = t9 * t15 * 3.0f;
        float t19 = t9 * t12 * 3.0f;
        float t20 = t10 * t11 * t15 * 3.0f;
        float t21 = t11 * t12 * t15 * 3.0f;
        float t22 = (t2 * t4 * t7 * t15) / 4.0f;
        float t24 = (dl_Beta_Hand * dl_Alpha_Hand * t2 * t4 * t10 * t15) / 2.0f;
        float t25 = (dl_Beta_Hand * dl_Alpha_Hand * t2 * t4 * t12 * t15) / 2.0f;
        float t26 = (t2 * t4 * t6 * t10 * t15) / 4.0f;
        float t28 = (t2 * t4 * t6 * t12 * t15) / 4.0f;
        float t23 = -t22;
        float t27 = t10 * t22;
        float t29 = t12 * t22;
        float t30 = t17 + t19 + 1.0f;
        float t32 = t24 + t25;
        float t35 = t15 + t18 + t20 + t21;
        float t31 = 1.0f / t30;
        float t36 = 1.0f / t35;
        float t42 = t16 + t23 + t26 + t27 + t28 + t29;
        float t33 = t3 * t5 * t9 * t14 * t31 * 1.2e+1f;
        float t37 = t2 * t3 * t4 * t8 * t36 * 1.2e+1f;
        float t38 = t2 * t4 * t5 * t8 * t36 * 1.2e+1f;
        float t41 = t3 * t5 * t8 * t11 * t36 * 1.2e+1f;
        float t34 = -t33;
        float t39 = -t37;
        float t40 = -t38;
        float t43 = t34 + t41;
        return new float[][] {
        new float[] {t9 * t10 * t14 * t31 * 1.2e+1f + t8 * t11 * t12 * t36 * 1.2e+1f, t43, t40, t43 },
        new float[] { t8 * t10 * t11 * t36 * 1.2e+1f + t9 * t12 * t14 * t31 * 1.2e+1f, t39, t40, t39 },
        new float[] {t8 * t9 * t36 * 1.2e+1f, -length_Hand * t2 * t5 * t6 - length_Hand * t2 * t5 * t7 - dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t3 * t4 * 2.0f + length_Hand * t4 * t5 * t36 * t42 * 1.2e+1f + t2 * t3 * t13 * t14 * t31 * t32 * 1.2e+1f, -length_Hand * t2 * t3 * t6 - length_Hand * t2 * t3 * t7 + dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t4 * t5 * 2.0f + length_Hand * t3 * t4 * t36 * t42 * 1.2e+1f - t2 * t5 * t13 * t14 * t31 * t32 * 1.2e+1f, -length_Hand * t4 * t7 - length_Hand * t2 * t36 * t42 * 1.2e+1f }
        };

    }

    public static float[][] find_Coeffs_Ddl_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float depth_Body, float dgamma_Body, float g, float gamma_Body, float height_Body, float m_Body, float width_Body)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        float t8 = g * m_Body;
        float t9 = Mathf.Pow(dalpha_Body, 2f);
        float t10 = Mathf.Pow(dbeta_Body, 2f);
        float t11 = Mathf.Pow(depth_Body, 2f);
        float t12 = Mathf.Pow(dgamma_Body, 2f);
        float t13 = Mathf.Pow(height_Body, 2f);
        float t14 = Mathf.Pow(height_Body, 3f);
        float t16 = Mathf.Pow(width_Body, 2f);
        float t22 = 1.0f / m_Body;
        float t15 = Mathf.Pow(t13, 2f);
        float t17 = Mathf.Pow(t2, 2f);
        float t18 = Mathf.Pow(t4, 2f);
        float t19 = Mathf.Pow(t5, 2f);
        float t20 = Mathf.Pow(t6, 2f);
        float t21 = Mathf.Pow(t7, 2f);
        float t23 = t5 * t6;
        float t24 = -t8;
        float t25 = m_Body * t11;
        float t26 = m_Body * t16;
        float t27 = t2 * t3;
        float t28 = t2 * t6;
        float t29 = t3 * t5;
        float t34 = t11 * t13;
        float t35 = t11 + t13;
        float t36 = t11 * t16;
        float t37 = t13 * t16;
        float t38 = t13 + t16;
        float t39 = dalpha_Body * dgamma_Body * height_Body * t5 * t7;
        float t42 = dalpha_Body * dgamma_Body * height_Body * m_Body * t2 * t7;
        float t45 = height_Body * t2 * t4 * t9 * 2.0f;
        float t46 = height_Body * t2 * t4 * t12 * 2.0f;
        float t51 = (height_Body * t2 * t4 * t8) / 2.0f;
        float t52 = (height_Body * t5 * t7 * t8) / 2.0f;
        float t54 = (height_Body * t2 * t4 * t9) / 2.0f;
        float t55 = (height_Body * t2 * t4 * t12) / 2.0f;
        float t66 = (height_Body * m_Body * t4 * t5 * t9) / 2.0f;
        float t67 = (height_Body * m_Body * t4 * t5 * t12) / 2.0f;
        float t74 = (m_Body * t4 * t7 * t12 * t13) / 4.0f;
        float t30 = t7 * t27;
        float t31 = t7 * t28;
        float t32 = t7 * t29;
        float t33 = t7 * t23;
        float t40 = t39 * 4.0f;
        float t41 = t13 * t18 * 3.0f;
        float t47 = -t39;
        float t49 = 1.0f / t35;
        float t50 = 1.0f / t38;
        float t53 = t25 + t26;
        float t56 = -t51;
        float t57 = -t52;
        float t58 = t2 * t5 * t15 * t18;
        float t59 = t2 * t5 * t15 * t21;
        float t62 = t15 * t17 * t18 * 3.0f;
        float t63 = t15 * t17 * t21 * 3.0f;
        float t64 = t15 * t18 * t19 * 3.0f;
        float t65 = t15 * t19 * t21 * 3.0f;
        float t72 = t2 * t5 * t21 * t34;
        float t73 = t2 * t5 * t18 * t37;
        float t75 = t17 * t21 * t34 * 3.0f;
        float t76 = t17 * t18 * t37 * 3.0f;
        float t78 = t19 * t21 * t34 * 3.0f;
        float t79 = t18 * t19 * t37 * 3.0f;
        float t80 = -t74;
        float t82 = (dalpha_Body * dgamma_Body * m_Body * t4 * t7 * t13 * t17) / 2.0f;
        float t83 = (dalpha_Body * dgamma_Body * m_Body * t4 * t7 * t13 * t19) / 2.0f;
        float t84 = (m_Body * t4 * t7 * t9 * t13 * t17) / 4.0f;
        float t85 = t17 * t74;
        float t86 = (m_Body * t4 * t7 * t9 * t13 * t19) / 4.0f;
        float t87 = t19 * t74;
        float t104 = t15 + t34 + t36 + t37;
        float t139 = t24 + t42 + t66 + t67;
        float t43 = -t31;
        float t44 = -t32;
        float t48 = -t40;
        float t60 = t23 + t30;
        float t61 = t27 + t33;
        float t68 = 1.0f / t53;
        float t69 = -t59;
        float t77 = t38 + t41;
        float t81 = -t72;
        float t88 = height_Body * t4 * t22 * t50 * 6.0f;
        float t89 = height_Body * t2 * t4 * t22 * t49 * 6.0f;
        float t90 = height_Body * t4 * t5 * t22 * t49 * 6.0f;
        float t91 = height_Body * t2 * t7 * t22 * t50 * 6.0f;
        float t92 = height_Body * t5 * t7 * t22 * t50 * 6.0f;
        float t93 = t3 * t7 * t22 * t50 * width_Body * 6.0f;
        float t97 = t4 * t7 * t12 * t13 * t50 * 3.0f;
        float t98 = t4 * t22 * t27 * t50 * width_Body * 6.0f;
        float t99 = t4 * t22 * t29 * t50 * width_Body * 6.0f;
        float t102 = height_Body * t3 * t4 * t7 * t22 * t50 * width_Body * 3.0f;
        float t105 = t2 * t4 * t7 * t13 * t22 * t50 * 3.0f;
        float t106 = t4 * t5 * t7 * t13 * t22 * t50 * 3.0f;
        float t108 = height_Body * t18 * t22 * t27 * t50 * width_Body * 3.0f;
        float t109 = height_Body * t21 * t22 * t27 * t50 * width_Body * 3.0f;
        float t110 = height_Body * t18 * t22 * t29 * t50 * width_Body * 3.0f;
        float t111 = height_Body * t21 * t22 * t29 * t50 * width_Body * 3.0f;
        float t116 = 1.0f / t104;
        float t117 = height_Body * t4 * t5 * t22 * t30 * t50 * width_Body * 3.0f;
        float t128 = t47 + t54 + t55;
        float t146 = t56 + t82 + t83;
        float t157 = height_Body * t5 * t7 * t22 * t50 * t139 * -6.0f;
        float t158 = height_Body * t2 * t4 * t22 * t49 * t139 * -6.0f;
        float t160 = t62 + t65 + t76 + t78 + t104;
        float t161 = t63 + t64 + t75 + t79 + t104;
        float t164 = t57 + t80 + t84 + t85 + t86 + t87;
        float t70 = t28 + t44;
        float t71 = t29 + t43;
        float t94 = -t89;
        float t95 = -t90;
        float t96 = -t93;
        float t100 = -t98;
        float t101 = -t99;
        float t103 = t3 * t4 * t23 * t68 * width_Body * 6.0f;
        float t107 = t4 * t6 * t27 * t68 * width_Body * 6.0f;
        float t112 = t2 * t4 * t7 * t20 * t68 * width_Body * 6.0f;
        float t113 = t4 * t5 * t7 * t20 * t68 * width_Body * 6.0f;
        float t114 = t16 * t18 * t20 * t68 * 3.0f;
        float t115 = t19 * t102;
        float t118 = t17 * t102;
        float t119 = -t117;
        float t120 = t22 * t49 * t60 * width_Body * 6.0f;
        float t121 = t45 + t46 + t48;
        float t122 = t22 * t50 * t77;
        float t124 = t27 * t61 * t68 * width_Body * 6.0f;
        float t125 = t29 * t61 * t68 * width_Body * 6.0f;
        float t126 = height_Body * t2 * t4 * t22 * t49 * t60 * width_Body * 3.0f;
        float t127 = height_Body * t4 * t5 * t22 * t49 * t60 * width_Body * 3.0f;
        float t129 = t31 * t61 * t68 * width_Body * 6.0f;
        float t130 = t33 * t61 * t68 * width_Body * 6.0f;
        float t133 = t4 * t6 * t16 * t61 * t68 * 3.0f;
        float t140 = t88 + t93;
        float t142 = t91 + t98;
        float t143 = t92 + t99;
        float t154 = t58 + t69 + t73 + t81;
        float t155 = t92 * t139;
        float t156 = t89 * t139;
        float t159 = t22 * t49 * t146 * 1.2e+1f;
        float t165 = t22 * t116 * t160;
        float t166 = t22 * t116 * t161;
        float t167 = t22 * t50 * t164 * 1.2e+1f;
        float t123 = t22 * t49 * t70 * width_Body * 6.0f;
        float t131 = t27 * t68 * t71 * width_Body * 6.0f;
        float t132 = t29 * t68 * t71 * width_Body * 6.0f;
        float t134 = height_Body * t2 * t4 * t22 * t49 * t70 * width_Body * 3.0f;
        float t135 = height_Body * t4 * t5 * t22 * t49 * t70 * width_Body * 3.0f;
        float t136 = t31 * t68 * t71 * width_Body * 6.0f;
        float t137 = t33 * t68 * t71 * width_Body * 6.0f;
        float t138 = t4 * t6 * t16 * t68 * t71 * 3.0f;
        float t141 = t88 + t96;
        float t144 = t91 + t100;
        float t145 = t92 + t101;
        float t147 = height_Body * t4 * t5 * t49 * t121 * (3.0f / 2.0f);
        float t148 = height_Body * t2 * t7 * t50 * t121 * (3.0f / 2.0f);
        float t150 = t89 + t120;
        float t151 = t94 + t120;
        float t162 = t22 * t116 * t154 * 3.0f;
        float t149 = -t148;
        float t152 = t90 + t123;
        float t153 = t95 + t123;
        float t163 = -t162;
        float t168 = t147 + t158 + t159;
        float t169 = t97 + t149 + t157 + t167;
        return new float[][]
        {
            new float[] {-t102 - t114 + t122 + (t3 * t7 * t141 * width_Body) / 2.0f, t105 - t109 - (width_Body * (-t103 + t112 + t4 * t27 * t141)) / 2.0f, t106 - t111 - (width_Body * (t107 + t113 + t4 * t29 * t141)) / 2.0f, t105 + t108 + t138 + (t3 * t7 * t142 * width_Body) / 2.0f },
            new float[] {t118 + t135 + t166 - (width_Body * (t132 - t136 + t28 * t152 + t44 * t152 + t4 * t27 * t142)) / 2.0f, t117 - t134 + t163 - (width_Body * (-t131 - t137 + t23 * t152 + t30 * t152 + t4 * t29 * t142)) / 2.0f, t106 + t110 - t133 + (t3 * t7 * t143 * width_Body) / 2.0f, t117 + t127 + t163 + (width_Body * (t125 - t129 + t28 * (t89 - t120) + t44 * (t89 - t120) - t4 * t27 * t143)) / 2.0f },
            new float[] {t115 - t126 + t165 - (width_Body * (t124 + t130 - t23 * (t89 - t120) - t30 * (t89 - t120) + t4 * t29 * t143)) / 2.0f, t102 + t114 + t122 + (t3 * t7 * t140 * width_Body) / 2.0f, t105 + t109 - (width_Body * (t103 - t112 + t4 * t27 * t140)) / 2.0f, t106 + t111 + (width_Body * (t107 + t113 - t4 * t29 * t140)) / 2.0f, t105 - t108 - t138 + (t3 * t7 * t144 * width_Body) / 2.0f, -t135 + t166 - (width_Body * (-t132 + t136 + t28 * (t90 - t123) + t44 * (t90 - t123) + t4 * t27 * t144)) / 2.0f - height_Body * t3 * t4 * t7 * t17 * t22 * t50 * width_Body * 3.0f, t119 + t134 + t163 - (width_Body * (t131 + t137 + t23 * (t90 - t123) + t30 * (t90 - t123) + t4 * t29 * t144)) / 2.0f, t106 - t110 + t133 + (t3 * t7 * t145 * width_Body) / 2.0f, t119 - t127 + t163 + (width_Body * (-t125 + t129 + t28 * t150 + t44 * t150 - t4 * t27 * t145)) / 2.0f, t126 + t165 + (width_Body * (t124 + t130 + t23 * t150 + t30 * t150 - t4 * t29 * t145)) / 2.0f - height_Body * t3 * t4 * t7 * t19 * t22 * t50 * width_Body * 3.0f, t106 * t139 + (t3 * t4 * t10 * width_Body) / 2.0f + (t3 * t4 * t12 * width_Body) / 2.0f - (t3 * t7 * t169 * width_Body) / 2.0f - dbeta_Body * dgamma_Body * t6 * t7 * width_Body - (height_Body * t7 * t12 * t50 * t77) / 2.0f - height_Body * t4 * t22 * t50 * t164 * 6.0f + t2 * t4 * t7 * t13 * t50 * t121 * (3.0f / 4.0f), (width_Body * (t9 * t23 + t10 * t23 + t9 * t30 + t10 * t30 + t12 * t30 - t28 * t168 + t32 * t168 - dalpha_Body * dbeta_Body * t27 * 2.0f - dalpha_Body * dbeta_Body * t33 * 2.0f + t4 * t27 * t169 + dalpha_Body * dgamma_Body * t4 * t29 * 2.0f + dbeta_Body * dgamma_Body * t4 * t28 * 2.0f)) / 2.0f + t90 * t146 + t116 * t128 * t161 - t22 * t116 * t139 * t154 * 3.0f - height_Body * t2 * t7 * t22 * t50 * t164 * 6.0f - t2 * t4 * t12 * t14 * t21 * t50 * (3.0f / 2.0f), width_Body * (t9 * t28 + t10 * t28 + t9 * t44 + t10 * t44 + t12 * t44 + t23 * t168 + t30 * t168 + dalpha_Body * dbeta_Body * t29 * 2.0f - dalpha_Body * dbeta_Body * t31 * 2.0f - t4 * t29 * t169 + dalpha_Body * dgamma_Body * t4 * t27 * 2.0f - dbeta_Body * dgamma_Body * t4 * t23 * 2.0f) * (-1.0f / 2.0f) + t139 * t165 - t116 * t128 * t154 * 3.0f - height_Body * t2 * t4 * t22 * t49 * t146 * 6.0f - height_Body * t5 * t7 * t22 * t50 * t164 * 6.0f - t4 * t5 * t12 * t14 * t21 * t50 * (3.0f / 2.0f) }
        };
    }

    public static float[][] find_Coeffs_Ddr_Arm_Bottom(float dr_Beta_Hand, float dr_Alpha_Hand, float g, float length_Hand, float m_Hand, float r_Alpha_Hand, float r_Beta_Hand)
    {

        float t2 = Mathf.Cos(r_Alpha_Hand);
        float t3 = Mathf.Cos(r_Beta_Hand);
        float t4 = Mathf.Sin(r_Alpha_Hand);
        float t5 = Mathf.Sin(r_Beta_Hand);
        float t6 = Mathf.Pow(dr_Beta_Hand, 2f);
        float t7 = Mathf.Pow(dr_Alpha_Hand, 2f);
        float t8 = Mathf.Pow(length_Hand, 2f);
        float t13 = 1.0f / length_Hand;
        float t14 = 1.0f / m_Hand;
        float t9 = Mathf.Pow(t2, 2f);
        float t10 = Mathf.Pow(t3, 2f);
        float t11 = Mathf.Pow(t4, 2f);
        float t12 = Mathf.Pow(t5, 2f);
        float t15 = m_Hand * t8;
        float t16 = (g * length_Hand * m_Hand * t2) / 2.0f;
        float t17 = t9 * t10 * 3.0f;
        float t18 = t9 * t15 * 3.0f;
        float t19 = t9 * t12 * 3.0f;
        float t20 = t10 * t11 * t15 * 3.0f;
        float t21 = t11 * t12 * t15 * 3.0f;
        float t22 = (t2 * t4 * t7 * t15) / 4.0f;
        float t24 = (dr_Beta_Hand * dr_Alpha_Hand * t2 * t4 * t10 * t15) / 2.0f;
        float t25 = (dr_Beta_Hand * dr_Alpha_Hand * t2 * t4 * t12 * t15) / 2.0f;
        float t26 = (t2 * t4 * t6 * t10 * t15) / 4.0f;
        float t28 = (t2 * t4 * t6 * t12 * t15) / 4.0f;
        float t23 = -t22;
        float t27 = t10 * t22;
        float t29 = t12 * t22;
        float t30 = t17 + t19 + 1.0f;
        float t32 = t24 + t25;
        float t34 = t15 + t18 + t20 + t21;
        float t31 = 1.0f / t30;
        float t35 = 1.0f / t34;
        float t41 = t16 + t23 + t26 + t27 + t28 + t29;
        float t33 = t3 * t5 * t9 * t14 * t31 * 1.2e+1f;
        float t36 = t2 * t3 * t4 * t8 * t35 * 1.2e+1f;
        float t37 = t2 * t4 * t5 * t8 * t35 * 1.2e+1f;
        float t39 = t3 * t5 * t8 * t11 * t35 * 1.2e+1f;
        float t38 = -t36;
        float t40 = -t39;
        float t42 = t33 + t40;
        return new float[][] {
            new float[] { t9 * t10 * t14 * t31 * 1.2e+1f + t8 * t11 * t12 * t35 * 1.2e+1f, t42, t37, t42 },
            new float[] { t8 * t10 * t11 * t35 * 1.2e+1f + t9 * t12 * t14 * t31 * 1.2e+1f, t38, t37, t38 },
            new float[] { t8 * t9 * t35 * 1.2e+1f, length_Hand * t2 * t5 * t6 + length_Hand * t2 * t5 * t7 + dr_Beta_Hand * dr_Alpha_Hand * length_Hand * t3 * t4 * 2.0f - length_Hand * t4 * t5 * t35 * t41 * 1.2e+1f - t2 * t3 * t13 * t14 * t31 * t32 * 1.2e+1f, -length_Hand * t2 * t3 * t6 - length_Hand * t2 * t3 * t7 + dr_Beta_Hand * dr_Alpha_Hand * length_Hand * t4 * t5 * 2.0f + length_Hand * t3 * t4 * t35 * t41 * 1.2e+1f - t2 * t5 * t13 * t14 * t31 * t32 * 1.2e+1f, -length_Hand * t4 * t7 - length_Hand * t2 * t35 * t41 * 1.2e+1f }
        };
    }

    public static float[][] find_Coeffs_Ddr_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float depth_Body, float dgamma_Body, float g, float gamma_Body, float height_Body, float m_Body, float width_Body)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        float t8 = g * m_Body;
        float t9 = Mathf.Pow(dalpha_Body, 2f);
        float t10 = Mathf.Pow(dbeta_Body, 2f);
        float t11 = Mathf.Pow(depth_Body, 2f);
        float t12 = Mathf.Pow(dgamma_Body, 2f);
        float t13 = Mathf.Pow(height_Body, 2f);
        float t14 = Mathf.Pow(height_Body, 3f);
        float t16 = Mathf.Pow(width_Body, 2f);
        float t22 = 1.0f / m_Body;
        float t15 = Mathf.Pow(t13, 2f);
        float t17 = Mathf.Pow(t2, 2f);
        float t18 = Mathf.Pow(t4, 2f);
        float t19 = Mathf.Pow(t5, 2f);
        float t20 = Mathf.Pow(t6, 2f);
        float t21 = Mathf.Pow(t7, 2f);
        float t23 = t5 * t6;
        float t24 = -t8;
        float t25 = m_Body * t11;
        float t26 = m_Body * t16;
        float t27 = t2 * t3;
        float t28 = t2 * t6;
        float t29 = t3 * t5;
        float t34 = t11 * t13;
        float t35 = t11 + t13;
        float t36 = t11 * t16;
        float t37 = t13 * t16;
        float t38 = t13 + t16;
        float t39 = dalpha_Body * dgamma_Body * height_Body * t5 * t7;
        float t42 = dalpha_Body * dgamma_Body * height_Body * m_Body * t2 * t7;
        float t45 = height_Body * t2 * t4 * t9 * 2.0f;
        float t46 = height_Body * t2 * t4 * t12 * 2.0f;
        float t51 = (height_Body * t2 * t4 * t8) / 2.0f;
        float t52 = (height_Body * t5 * t7 * t8) / 2.0f;
        float t54 = (height_Body * t2 * t4 * t9) / 2.0f;
        float t55 = (height_Body * t2 * t4 * t12) / 2.0f;
        float t66 = (height_Body * m_Body * t4 * t5 * t9) / 2.0f;
        float t67 = (height_Body * m_Body * t4 * t5 * t12) / 2.0f;
        float t74 = (m_Body * t4 * t7 * t12 * t13) / 4.0f;
        float t30 = t7 * t27;
        float t31 = t7 * t28;
        float t32 = t7 * t29;
        float t33 = t7 * t23;
        float t40 = t39 * 4.0f;
        float t41 = t13 * t18 * 3.0f;
        float t47 = -t39;
        float t49 = 1.0f / t35;
        float t50 = 1.0f / t38;
        float t53 = t25 + t26;
        float t56 = -t51;
        float t57 = -t52;
        float t58 = t2 * t5 * t15 * t18;
        float t59 = t2 * t5 * t15 * t21;
        float t62 = t15 * t17 * t18 * 3.0f;
        float t63 = t15 * t17 * t21 * 3.0f;
        float t64 = t15 * t18 * t19 * 3.0f;
        float t65 = t15 * t19 * t21 * 3.0f;
        float t72 = t2 * t5 * t21 * t34;
        float t73 = t2 * t5 * t18 * t37;
        float t75 = t17 * t21 * t34 * 3.0f;
        float t76 = t17 * t18 * t37 * 3.0f;
        float t78 = t19 * t21 * t34 * 3.0f;
        float t79 = t18 * t19 * t37 * 3.0f;
        float t80 = -t74;
        float t82 = (dalpha_Body * dgamma_Body * m_Body * t4 * t7 * t13 * t17) / 2.0f;
        float t83 = (dalpha_Body * dgamma_Body * m_Body * t4 * t7 * t13 * t19) / 2.0f;
        float t84 = (m_Body * t4 * t7 * t9 * t13 * t17) / 4.0f;
        float t85 = t17 * t74;
        float t86 = (m_Body * t4 * t7 * t9 * t13 * t19) / 4.0f;
        float t87 = t19 * t74;
        float t104 = t15 + t34 + t36 + t37;
        float t139 = t24 + t42 + t66 + t67;
        float t43 = -t31;
        float t44 = -t32;
        float t48 = -t40;
        float t60 = t23 + t30;
        float t61 = t27 + t33;
        float t68 = 1.0f / t53;
        float t69 = -t59;
        float t77 = t38 + t41;
        float t81 = -t72;
        float t88 = height_Body * t4 * t22 * t50 * 6.0f;
        float t89 = height_Body * t2 * t4 * t22 * t49 * 6.0f;
        float t90 = height_Body * t4 * t5 * t22 * t49 * 6.0f;
        float t91 = height_Body * t2 * t7 * t22 * t50 * 6.0f;
        float t92 = height_Body * t5 * t7 * t22 * t50 * 6.0f;
        float t93 = t3 * t7 * t22 * t50 * width_Body * 6.0f;
        float t97 = t4 * t7 * t12 * t13 * t50 * 3.0f;
        float t98 = t4 * t22 * t27 * t50 * width_Body * 6.0f;
        float t99 = t4 * t22 * t29 * t50 * width_Body * 6.0f;
        float t102 = height_Body * t3 * t4 * t7 * t22 * t50 * width_Body * 3.0f;
        float t105 = t2 * t4 * t7 * t13 * t22 * t50 * 3.0f;
        float t106 = t4 * t5 * t7 * t13 * t22 * t50 * 3.0f;
        float t108 = height_Body * t18 * t22 * t27 * t50 * width_Body * 3.0f;
        float t109 = height_Body * t21 * t22 * t27 * t50 * width_Body * 3.0f;
        float t110 = height_Body * t18 * t22 * t29 * t50 * width_Body * 3.0f;
        float t111 = height_Body * t21 * t22 * t29 * t50 * width_Body * 3.0f;
        float t116 = 1.0f / t104;
        float t117 = height_Body * t4 * t5 * t22 * t30 * t50 * width_Body * 3.0f;
        float t128 = t47 + t54 + t55;
        float t146 = t56 + t82 + t83;
        float t157 = height_Body * t5 * t7 * t22 * t50 * t139 * -6.0f;
        float t158 = height_Body * t2 * t4 * t22 * t49 * t139 * -6.0f;
        float t160 = t62 + t65 + t76 + t78 + t104;
        float t161 = t63 + t64 + t75 + t79 + t104;
        float t164 = t57 + t80 + t84 + t85 + t86 + t87;
        float t70 = t28 + t44;
        float t71 = t29 + t43;
        float t94 = -t89;
        float t95 = -t90;
        float t96 = -t93;
        float t100 = -t98;
        float t101 = -t99;
        float t103 = t3 * t4 * t23 * t68 * width_Body * 6.0f;
        float t107 = t4 * t6 * t27 * t68 * width_Body * 6.0f;
        float t112 = t2 * t4 * t7 * t20 * t68 * width_Body * 6.0f;
        float t113 = t4 * t5 * t7 * t20 * t68 * width_Body * 6.0f;
        float t114 = t16 * t18 * t20 * t68 * 3.0f;
        float t115 = t19 * t102;
        float t118 = t17 * t102;
        float t119 = -t117;
        float t120 = t22 * t49 * t60 * width_Body * 6.0f;
        float t121 = t45 + t46 + t48;
        float t122 = t22 * t50 * t77;
        float t124 = t27 * t61 * t68 * width_Body * 6.0f;
        float t125 = t29 * t61 * t68 * width_Body * 6.0f;
        float t126 = height_Body * t2 * t4 * t22 * t49 * t60 * width_Body * 3.0f;
        float t127 = height_Body * t4 * t5 * t22 * t49 * t60 * width_Body * 3.0f;
        float t129 = t31 * t61 * t68 * width_Body * 6.0f;
        float t130 = t33 * t61 * t68 * width_Body * 6.0f;
        float t133 = t4 * t6 * t16 * t61 * t68 * 3.0f;
        float t140 = t88 + t93;
        float t142 = t91 + t98;
        float t143 = t92 + t99;
        float t154 = t58 + t69 + t73 + t81;
        float t155 = t92 * t139;
        float t156 = t89 * t139;
        float t159 = t22 * t49 * t146 * 1.2e+1f;
        float t165 = t22 * t116 * t160;
        float t166 = t22 * t116 * t161;
        float t167 = t22 * t50 * t164 * 1.2e+1f;
        float t123 = t22 * t49 * t70 * width_Body * 6.0f;
        float t131 = t27 * t68 * t71 * width_Body * 6.0f;
        float t132 = t29 * t68 * t71 * width_Body * 6.0f;
        float t134 = height_Body * t2 * t4 * t22 * t49 * t70 * width_Body * 3.0f;
        float t135 = height_Body * t4 * t5 * t22 * t49 * t70 * width_Body * 3.0f;
        float t136 = t31 * t68 * t71 * width_Body * 6.0f;
        float t137 = t33 * t68 * t71 * width_Body * 6.0f;
        float t138 = t4 * t6 * t16 * t68 * t71 * 3.0f;
        float t141 = t88 + t96;
        float t144 = t91 + t100;
        float t145 = t92 + t101;
        float t147 = height_Body * t4 * t5 * t49 * t121 * (3.0f / 2.0f);
        float t148 = height_Body * t2 * t7 * t50 * t121 * (3.0f / 2.0f);
        float t150 = t89 + t120;
        float t151 = t94 + t120;
        float t162 = t22 * t116 * t154 * 3.0f;
        float t149 = -t148;
        float t152 = t90 + t123;
        float t153 = t95 + t123;
        float t163 = -t162;
        float t168 = t147 + t158 + t159;
        float t169 = t97 + t149 + t157 + t167;
        return new float[][] {
            new float[] {-t102 + t114 + t122 - (t3 * t7 * t141 * width_Body) / 2.0f, t105 - t109 + (width_Body * (-t103 + t112 + t4 * t27 * t141)) / 2.0f, t106 - t111 + (width_Body * (t107 + t113 + t4 * t29 * t141)) / 2.0f, t105 + t108 - t138 - (t3 * t7 * t142 * width_Body) / 2.0f },
            new float[] {t118 + t135 + t166 + (width_Body * (t132 - t136 + t28 * t152 + t44 * t152 + t4 * t27 * t142)) / 2.0f, t117 - t134 + t163 + (width_Body * (-t131 - t137 + t23 * t152 + t30 * t152 + t4 * t29 * t142)) / 2.0f, t106 + t110 + t133 - (t3 * t7 * t143 * width_Body) / 2.0f, t117 + t127 + t163 - (width_Body * (t125 - t129 + t28 * (t89 - t120) + t44 * (t89 - t120) - t4 * t27 * t143)) / 2.0f },
            new float[] {t115 - t126 + t165 + (width_Body * (t124 + t130 - t23 * (t89 - t120) - t30 * (t89 - t120) + t4 * t29 * t143)) / 2.0f, t102 - t114 + t122 - (t3 * t7 * t140 * width_Body) / 2.0f, t105 + t109 + (width_Body * (t103 - t112 + t4 * t27 * t140)) / 2.0f, t106 + t111 - (width_Body * (t107 + t113 - t4 * t29 * t140)) / 2.0f, t105 - t108 + t138 - (t3 * t7 * t144 * width_Body) / 2.0f, -t135 + t166 + (width_Body * (-t132 + t136 + t28 * (t90 - t123) + t44 * (t90 - t123) + t4 * t27 * t144)) / 2.0f - height_Body * t3 * t4 * t7 * t17 * t22 * t50 * width_Body * 3.0f, t119 + t134 + t163 + (width_Body * (t131 + t137 + t23 * (t90 - t123) + t30 * (t90 - t123) + t4 * t29 * t144)) / 2.0f, t106 - t110 - t133 - (t3 * t7 * t145 * width_Body) / 2.0f, t119 - t127 + t163 - (width_Body * (-t125 + t129 + t28 * t150 + t44 * t150 - t4 * t27 * t145)) / 2.0f, t126 + t165 - (width_Body * (t124 + t130 + t23 * t150 + t30 * t150 - t4 * t29 * t145)) / 2.0f - height_Body * t3 * t4 * t7 * t19 * t22 * t50 * width_Body * 3.0f, t106* t139 -(t3 * t4 * t10 * width_Body) / 2.0f - (t3 * t4 * t12 * width_Body) / 2.0f + (t3 * t7 * t169 * width_Body) / 2.0f + dbeta_Body * dgamma_Body * t6 * t7 * width_Body - (height_Body * t7 * t12 * t50 * t77) / 2.0f - height_Body * t4 * t22 * t50 * t164 * 6.0f + t2 * t4 * t7 * t13 * t50 * t121 * (3.0f / 4.0f), width_Body * (t9 * t23 + t10 * t23 + t9 * t30 + t10 * t30 + t12 * t30 - t28 * t168 + t32 * t168 - dalpha_Body * dbeta_Body * t27 * 2.0f - dalpha_Body * dbeta_Body * t33 * 2.0f + t4 * t27 * t169 + dalpha_Body * dgamma_Body * t4 * t29 * 2.0f + dbeta_Body * dgamma_Body * t4 * t28 * 2.0f) * (-1.0f / 2.0f) + t90 * t146 + t116 * t128 * t161 - t22 * t116 * t139 * t154 * 3.0f - height_Body * t2 * t7 * t22 * t50 * t164 * 6.0f - t2 * t4 * t12 * t14 * t21 * t50 * (3.0f / 2.0f), (width_Body * (t9 * t28 + t10 * t28 + t9 * t44 + t10 * t44 + t12 * t44 + t23 * t168 + t30 * t168 + dalpha_Body * dbeta_Body * t29 * 2.0f - dalpha_Body * dbeta_Body * t31 * 2.0f - t4 * t29 * t169 + dalpha_Body * dgamma_Body * t4 * t27 * 2.0f - dbeta_Body * dgamma_Body * t4 * t23 * 2.0f)) / 2.0f + t139 * t165 - t116 * t128 * t154 * 3.0f - height_Body * t2 * t4 * t22 * t49 * t146 * 6.0f - height_Body * t5 * t7 * t22 * t50 * t164 * 6.0f - t4 * t5 * t12 * t14 * t21 * t50 * (3.0f / 2.0f) }
            };

    }

    public static float[][] find_L_Arm_Bottom(float l_Alpha_Hand, float l_Beta_Hand, float l_X_Fixed, float l_Y_Fixed, float l_Z_Fixed, float length_Hand)
    {

        float t2 = Mathf.Cos(l_Alpha_Hand);
        return new float[][] {
            new float[] {l_X_Fixed + length_Hand * t2 * Mathf.Sin(l_Beta_Hand), l_Y_Fixed + length_Hand * t2 * Mathf.Cos(l_Beta_Hand), l_Z_Fixed + length_Hand * Mathf.Sin(l_Alpha_Hand) }
        };
    }
    public static float[][] find_L_Hip(float alpha_Body, float beta_Body, float gamma_Body, float height_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        return new float[][] {
            new float[] {x_Head - height_Body * t7 - (t3 * t4 * width_Body) / 2.0f, y_Head - (width_Body * (t5 * t6 + t2 * t3 * t7)) / 2.0f + height_Body * t2 * t4, z_Head + (width_Body * (t2 * t6 - t3 * t5 * t7)) / 2.0f + height_Body * t4 * t5 }
        };
    }

    public static float[][] find_L_Shoulder(float alpha_Body, float beta_Body, float gamma_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Sin(alpha_Body);
        float t5 = Mathf.Sin(beta_Body);
        float t6 = Mathf.Sin(gamma_Body);
        return new float[][] {
            new float[] {x_Head - (t3 * width_Body * Mathf.Cos(gamma_Body)) / 2.0f, y_Head - (width_Body * (t4 * t5 + t2 * t3 * t6)) / 2.0f, z_Head + (width_Body * (t2 * t5 - t3 * t4 * t6)) / 2.0f }
        };
    }

    public static float[][] find_R_Arm_Bottom(float length_Hand, float r_Alpha_Hand, float r_Beta_Hand, float r_X_Fixed, float r_Y_Fixed, float r_Z_Fixed)
    {
        float t2 = Mathf.Cos(r_Alpha_Hand);
        return new float[][] {
            new float[] {r_X_Fixed - length_Hand * t2 * Mathf.Sin(r_Beta_Hand), r_Y_Fixed + length_Hand * t2 * Mathf.Cos(r_Beta_Hand), r_Z_Fixed + length_Hand * Mathf.Sin(r_Alpha_Hand) }
        };
    }

    public static float[][] find_R_Hip(float alpha_Body, float beta_Body, float gamma_Body, float height_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        return new float[][] {
            new float[] { x_Head - height_Body * t7 + (t3 * t4 * width_Body) / 2.0f, y_Head + (width_Body * (t5 * t6 + t2 * t3 * t7)) / 2.0f + height_Body * t2 * t4, z_Head - (width_Body * (t2 * t6 - t3 * t5 * t7)) / 2.0f + height_Body * t4 * t5 }
        };
    }

    public static float[][] find_R_Shoulder(float alpha_Body, float beta_Body, float gamma_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Sin(alpha_Body);
        float t5 = Mathf.Sin(beta_Body);
        float t6 = Mathf.Sin(gamma_Body);
        return new float[][] {
            new float[] {x_Head + (t3 * width_Body * Mathf.Cos(gamma_Body)) / 2.0f, y_Head + (width_Body * (t4 * t5 + t2 * t3 * t6)) / 2.0f, z_Head - (width_Body * (t2 * t5 - t3 * t4 * t6)) / 2.0f }
        };
    }


}
