using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NMC
{
    public static Matrix FFD_Coeffs_Ddl_Arm_Bottom(float dl_Beta_Hand, float dl_Alpha_Hand, float g, float l_Alpha_Hand, float l_Beta_Hand, float l_Tau_Beta_Shoulder, float l_Tau_Alpha_Shoulder, float length_Hand, float m_Hand)
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
        float t21 = (g * length_Hand * m_Hand * t2 * t3) / 2.0f;
        float t22 = (g * length_Hand * m_Hand * t4 * t5) / 2.0f;
        float t16 = l_Tau_Beta_Shoulder * t11;
        float t17 = l_Tau_Beta_Shoulder * t9;
        float t18 = t9 * t10 * 3.0f;
        float t19 = t10 * t15 * 3.0f;
        float t20 = t10 * t11 * 3.0f;
        float t23 = -t22;
        float t24 = t9 * t12 * t15 * 3.0f;
        float t25 = t11 * t12 * t15 * 3.0f;
        float t26 = (t3 * t5 * t6 * t15) / 4.0f;
        float t28 = (dl_Beta_Hand * dl_Alpha_Hand * t3 * t5 * t9 * t15) / 2.0f;
        float t29 = (dl_Beta_Hand * dl_Alpha_Hand * t3 * t5 * t11 * t15) / 2.0f;
        float t31 = (t3 * t5 * t7 * t9 * t15) / 4.0f;
        float t34 = (t3 * t5 * t7 * t11 * t15) / 4.0f;
        float t27 = -t26;
        float t30 = t9 * t26;
        float t32 = -t28;
        float t33 = t11 * t26;
        float t35 = -t29;
        float t36 = t18 + t20 + 1.0f;
        float t39 = t15 + t19 + t24 + t25;
        float t37 = 1.0f / t36;
        float t40 = 1.0f / t39;
        float A11 = t8 * t10 * t40 * -1.2e+1f;
        float t41 = l_Tau_Alpha_Shoulder + t21 + t32 + t35;
        float t47 = t16 + t17 + t23 + t27 + t30 + t31 + t33 + t34;
        float t38 = t2 * t4 * t10 * t14 * t37 * 1.2e+1f;
        float t42 = t2 * t3 * t5 * t8 * t40 * 1.2e+1f;
        float A12 = t42;
        float t43 = t3 * t4 * t5 * t8 * t40 * 1.2e+1f;
        float A13 = t43;
        float A14 = -length_Hand * t5 * t6 - length_Hand * t3 * t40 * t47 * 1.2e+1f;
        float A21 = t42;
        float A22 = t8 * t9 * t12 * t40 * -1.2e+1f - t10 * t11 * t14 * t37 * 1.2e+1f;
        float t44 = t2 * t4 * t8 * t12 * t40 * 1.2e+1f;
        float t45 = -t44;
        float t46 = t38 + t45;
        float A23 = t46;
        float A24 = -length_Hand * t2 * t3 * t6 - length_Hand * t2 * t3 * t7 + dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t4 * t5 * 2.0f + length_Hand * t2 * t5 * t40 * t47 * 1.2e+1f + t3 * t4 * t13 * t14 * t37 * t41 * 1.2e+1f;
        float A31 = t43;
        float A32 = t46;
        float A33 = t9 * t10 * t14 * t37 * -1.2e+1f - t8 * t11 * t12 * t40 * 1.2e+1f;
        float A34 = -length_Hand * t3 * t4 * t6 - length_Hand * t3 * t4 * t7 - dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t2 * t5 * 2.0f + length_Hand * t4 * t5 * t40 * t47 * 1.2e+1f - t2 * t3 * t13 * t14 * t37 * t41 * 1.2e+1f;

        return new Matrix(new float[,]
        {
            { 0, 0, 0, A11, A12, A13, A14 },
            { 0, 0, 0, A21, A22, A23, A24 },
            { 0, 0, 0, A31, A32, A33, A34 }
        });
    }

    public static Matrix FFD_Coeffs_Ddl_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float depth_Body, float dgamma_Body, float g, float gamma_Body, float height_Body, float l_Alpha_Hand, float l_Tau_Beta_Shoulder, float l_Tau_Alpha_Shoulder, float m_Body, float r_Alpha_Hand, float r_Tau_Beta_Shoulder, float r_Tau_Alpha_Shoulder, float width_Body)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Cos(l_Alpha_Hand);
        float t6 = Mathf.Sin(alpha_Body);
        float t7 = Mathf.Cos(r_Alpha_Hand);
        float t8 = Mathf.Sin(beta_Body);
        float t9 = Mathf.Sin(gamma_Body);
        float t10 = Mathf.Sin(l_Alpha_Hand);
        float t11 = Mathf.Sin(r_Alpha_Hand);
        float t12 = g * m_Body;
        float t13 = l_Tau_Alpha_Shoulder + r_Tau_Alpha_Shoulder;
        float t14 = Mathf.Pow(dalpha_Body, 2f);
        float t15 = Mathf.Pow(dbeta_Body, 2f);
        float t16 = Mathf.Pow(depth_Body, 2f);
        float t17 = Mathf.Pow(dgamma_Body, 2f);
        float t18 = Mathf.Pow(height_Body, 2f);
        float t19 = Mathf.Pow(height_Body, 3f);
        float t21 = Mathf.Pow(width_Body, 2f);
        float t31 = 1.0f / m_Body;
        float t20 = Mathf.Pow(t18, 2f);
        float t22 = Mathf.Pow(t2, 2f);
        float t23 = Mathf.Pow(t4, 2f);
        float t24 = Mathf.Pow(t6, 2f);
        float t25 = Mathf.Pow(t8, 2f);
        float t26 = Mathf.Pow(t9, 2f);
        float t27 = l_Tau_Beta_Shoulder * t5;
        float t28 = r_Tau_Beta_Shoulder * t7;
        float t29 = l_Tau_Beta_Shoulder * t10;
        float t30 = r_Tau_Beta_Shoulder * t11;
        float t32 = -t12;
        float t33 = m_Body * t16;
        float t34 = m_Body * t21;
        float t35 = t9 * t13;
        float t36 = t16 * t18;
        float t37 = t16 + t18;
        float t38 = t16 * t21;
        float t39 = t18 * t21;
        float t40 = t18 + t21;
        float t41 = dalpha_Body * dgamma_Body * height_Body * t6 * t9;
        float t44 = (t2 * t3 * width_Body) / 2.0f;
        float t45 = (t2 * t8 * width_Body) / 2.0f;
        float t46 = (t3 * t6 * width_Body) / 2.0f;
        float t47 = (t6 * t8 * width_Body) / 2.0f;
        float t48 = dalpha_Body * dgamma_Body * height_Body * m_Body * t2 * t9;
        float t51 = height_Body * t2 * t4 * t14 * 2.0f;
        float t52 = height_Body * t2 * t4 * t17 * 2.0f;
        float t57 = (height_Body * t2 * t4 * t12) / 2.0f;
        float t59 = (height_Body * t6 * t9 * t12) / 2.0f;
        float t64 = (height_Body * t2 * t4 * t14) / 2.0f;
        float t65 = (height_Body * t2 * t4 * t17) / 2.0f;
        float t67 = t2 * t8 * t9 * width_Body * (-1.0f / 2.0f);
        float t68 = t3 * t6 * t9 * width_Body * (-1.0f / 2.0f);
        float t77 = (height_Body * m_Body * t4 * t6 * t14) / 2.0f;
        float t78 = (height_Body * m_Body * t4 * t6 * t17) / 2.0f;
        float t85 = (m_Body * t4 * t9 * t17 * t18) / 4.0f;
        float t42 = t41 * 4.0f;
        float t43 = t18 * t23 * 3.0f;
        float t49 = t27 + t28;
        float t50 = t29 + t30;
        float t53 = -t41;
        float t55 = 1.0f / t37;
        float t56 = 1.0f / t40;
        float t58 = t9 * t44;
        float t60 = t9 * t45;
        float t61 = t9 * t46;
        float t62 = t9 * t47;
        float t63 = t33 + t34;
        float t66 = -t57;
        float t70 = t2 * t6 * t20 * t23;
        float t72 = t2 * t6 * t20 * t26;
        float t73 = t20 * t22 * t23 * 3.0f;
        float t74 = t20 * t22 * t26 * 3.0f;
        float t75 = t20 * t23 * t24 * 3.0f;
        float t76 = t20 * t24 * t26 * 3.0f;
        float t83 = t2 * t6 * t26 * t36;
        float t84 = t2 * t6 * t23 * t39;
        float t86 = t22 * t26 * t36 * 3.0f;
        float t87 = t22 * t23 * t39 * 3.0f;
        float t89 = t24 * t26 * t36 * 3.0f;
        float t90 = t23 * t24 * t39 * 3.0f;
        float t93 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t18 * t22) / 2.0f;
        float t94 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t18 * t24) / 2.0f;
        float t95 = (m_Body * t4 * t9 * t14 * t18 * t22) / 4.0f;
        float t96 = t22 * t85;
        float t97 = (m_Body * t4 * t9 * t14 * t18 * t24) / 4.0f;
        float t98 = t24 * t85;
        float t101 = m_Body * t4 * t9 * t17 * t18 * t22 * (-1.0f / 4.0f);
        float t103 = m_Body * t4 * t9 * t17 * t18 * t24 * (-1.0f / 4.0f);
        float t111 = t45 + t68;
        float t112 = t46 + t67;
        float t123 = t20 + t36 + t38 + t39;
        float t146 = t32 + t48 + t77 + t78;
        float t54 = -t42;
        float t69 = t2 * t49;
        float t71 = t6 * t50;
        float t79 = t4 * t6 * t49;
        float t80 = t2 * t4 * t50;
        float t81 = 1.0f / t63;
        float t82 = -t72;
        float t88 = t40 + t43;
        float t92 = -t83;
        float t99 = height_Body * t4 * t31 * t56 * 6.0f;
        float t100 = -t95;
        float t102 = -t97;
        float t104 = t44 + t62;
        float t105 = t47 + t58;
        float t106 = height_Body * t2 * t4 * t31 * t55 * 6.0f;
        float t107 = height_Body * t4 * t6 * t31 * t55 * 6.0f;
        float t108 = height_Body * t2 * t9 * t31 * t56 * 6.0f;
        float t109 = height_Body * t6 * t9 * t31 * t56 * 6.0f;
        float t110 = t3 * t9 * t31 * t56 * width_Body * 6.0f;
        float t116 = t4 * t9 * t17 * t18 * t56 * 3.0f;
        float t117 = t2 * t3 * t4 * t31 * t56 * width_Body * 6.0f;
        float t118 = t3 * t4 * t6 * t31 * t56 * width_Body * 6.0f;
        float t122 = height_Body * t3 * t4 * t9 * t31 * t56 * width_Body * 3.0f;
        float t124 = t2 * t4 * t9 * t18 * t31 * t56 * 3.0f;
        float t125 = t4 * t6 * t9 * t18 * t31 * t56 * 3.0f;
        float t126 = height_Body * t2 * t3 * t23 * t31 * t56 * width_Body * 3.0f;
        float t127 = height_Body * t2 * t3 * t26 * t31 * t56 * width_Body * 3.0f;
        float t128 = height_Body * t3 * t6 * t23 * t31 * t56 * width_Body * 3.0f;
        float t129 = height_Body * t3 * t6 * t26 * t31 * t56 * width_Body * 3.0f;
        float t134 = 1.0f / t123;
        float t137 = height_Body * t2 * t3 * t4 * t6 * t9 * t31 * t56 * width_Body * -3.0f;
        float t142 = t53 + t64 + t65;
        float t145 = t31 * t55 * t111 * 1.2e+1f;
        float t169 = t13 + t66 + t93 + t94;
        float t177 = height_Body * t2 * t4 * t31 * t55 * t146 * -6.0f;
        float t179 = t73 + t76 + t87 + t89 + t123;
        float t180 = t74 + t75 + t86 + t90 + t123;
        float t91 = -t79;
        float t113 = -t106;
        float t114 = -t107;
        float t115 = -t110;
        float t119 = -t116;
        float t120 = -t117;
        float t121 = -t118;
        float t130 = t2 * t3 * t4 * t8 * t21 * t81 * 3.0f;
        float t131 = t3 * t4 * t6 * t8 * t21 * t81 * 3.0f;
        float t132 = t21 * t23 * t25 * t81 * 3.0f;
        float t133 = t24 * t122;
        float t135 = t2 * t6 * t122;
        float t136 = t22 * t122;
        float t138 = t2 * t4 * t9 * t21 * t25 * t81 * 3.0f;
        float t139 = t4 * t6 * t9 * t21 * t25 * t81 * 3.0f;
        float t140 = t51 + t52 + t54;
        float t141 = t31 * t56 * t88;
        float t144 = t31 * t55 * t105 * 1.2e+1f;
        float t147 = t99 + t110;
        float t148 = t2 * t3 * t81 * t104 * width_Body * 6.0f;
        float t149 = t3 * t6 * t81 * t104 * width_Body * 6.0f;
        float t150 = t4 * t8 * t81 * t104 * width_Body * 6.0f;
        float t151 = t105 * t106;
        float t152 = t105 * t107;
        float t154 = t2 * t3 * t81 * t112 * width_Body * 6.0f;
        float t155 = t3 * t6 * t81 * t112 * width_Body * 6.0f;
        float t156 = t4 * t8 * t81 * t112 * width_Body * 6.0f;
        float t157 = t106 * t111;
        float t158 = t107 * t111;
        float t159 = t2 * t8 * t9 * t81 * t104 * width_Body * 6.0f;
        float t160 = t6 * t8 * t9 * t81 * t104 * width_Body * 6.0f;
        float t161 = t108 + t117;
        float t162 = t2 * t8 * t9 * t81 * t112 * width_Body * 6.0f;
        float t163 = t109 + t118;
        float t164 = t6 * t8 * t9 * t81 * t112 * width_Body * 6.0f;
        float t170 = t70 + t82 + t84 + t92;
        float t173 = t107 + t145;
        float t174 = t109 * t146;
        float t176 = t106 * t146;
        float t178 = t31 * t55 * t169 * 1.2e+1f;
        float t183 = t31 * t134 * t179;
        float t184 = t31 * t134 * t180;
        float t185 = t59 + t69 + t71 + t85 + t100 + t101 + t102 + t103;
        float t143 = t35 + t80 + t91;
        float t153 = t99 + t115;
        float A11 = -t122 - t132 + t141 + (t3 * t9 * t153 * width_Body) / 2.0f;
        float A12 = t124 + t126 + t156 + (t3 * t9 * t161 * width_Body) / 2.0f;
        float A13 = t125 + t128 - t150 + (t3 * t9 * t163 * width_Body) / 2.0f;
        float A14 = t122 + t132 + t141 + (t3 * t9 * t147 * width_Body) / 2.0f;
        float t165 = t108 + t120;
        float A15 = t124 - t126 - t156 + (t3 * t9 * t165 * width_Body) / 2.0f;
        float t166 = t109 + t121;
        float A16 = t125 - t128 + t150 + (t3 * t9 * t166 * width_Body) / 2.0f;
        float t167 = height_Body * t4 * t6 * t55 * t140 * (3.0f / 2.0f);
        float t168 = height_Body * t2 * t9 * t56 * t140 * (3.0f / 2.0f);
        float t171 = t106 + t144;
        float t172 = t113 + t144;
        float t175 = t114 + t145;
        float t181 = t31 * t134 * t170 * 3.0f;
        float t186 = t31 * t56 * t185 * 1.2e+1f;
        float t182 = -t181;
        float t187 = t167 + t177 + t178;
        float t188 = t119 + t168 + t174 + t186;
        float A17 = t125 * t146 + t99 * t185 + (t3 * t4 * t15 * width_Body) / 2.0f + (t3 * t4 * t17 * width_Body) / 2.0f + (t3 * t9 * t188 * width_Body) / 2.0f - dbeta_Body * dgamma_Body * t8 * t9 * width_Body - (height_Body * t9 * t17 * t56 * t88) / 2.0f - t4 * t8 * t81 * t143 * width_Body * 6.0f + t2 * t4 * t9 * t18 * t56 * t140 * (3.0f / 4.0f);
        float A21 = t124 - t127 + t131 - t138 - (t2 * t3 * t4 * t153 * width_Body) / 2.0f;
        float A22 = t136 - t155 + t158 + t162 + t184 + t61 * t173 - (t2 * t8 * t173 * width_Body) / 2.0f - (t2 * t3 * t4 * t161 * width_Body) / 2.0f;
        float A23 = t135 + t149 + t152 - t159 + t182 - t61 * (t106 - t144) + (t2 * t8 * width_Body * (t106 - t144)) / 2.0f - (t2 * t3 * t4 * t163 * width_Body) / 2.0f;
        float A24 = t124 + t127 - t131 + t138 - (t2 * t3 * t4 * t147 * width_Body) / 2.0f;
        float A25 = t155 - t162 + t184 - t45 * (t107 - t145) - t68 * (t107 - t145) - (t2 * t3 * t4 * t165 * width_Body) / 2.0f - height_Body * t4 * t6 * t31 * t55 * t111 * 6.0f - height_Body * t3 * t4 * t9 * t22 * t31 * t56 * width_Body * 3.0f;
        float A26 = t137 - t149 + t159 + t182 + t45 * t171 + t68 * t171 - (t2 * t3 * t4 * t166 * width_Body) / 2.0f - height_Body * t4 * t6 * t31 * t55 * t105 * 6.0f;
        float A27 = t14 * t47 + t15 * t47 + t14 * t58 + t15 * t58 + t17 * t58 + t61 * t187 + t107 * t169 + t108 * t185 + t134 * t142 * t180 - t31 * t134 * t146 * t170 * 3.0f - (t2 * t8 * t187 * width_Body) / 2.0f - dalpha_Body * dbeta_Body * t2 * t3 * width_Body - (t2 * t3 * t4 * t188 * width_Body) / 2.0f + t3 * t6 * t81 * t143 * width_Body * 6.0f - t2 * t4 * t17 * t19 * t26 * t56 * (3.0f / 2.0f) - t2 * t8 * t9 * t81 * t143 * width_Body * 6.0f - dalpha_Body * dbeta_Body * t6 * t8 * t9 * width_Body + dalpha_Body * dgamma_Body * t3 * t4 * t6 * width_Body + dbeta_Body * dgamma_Body * t2 * t4 * t8 * width_Body;
        float A31 = t125 - t129 - t130 - t139 - (t3 * t4 * t6 * t153 * width_Body) / 2.0f;
        float A32 = t135 + t154 + t164 + t182 - (t6 * t8 * t173 * width_Body) / 2.0f - (t3 * t4 * t6 * t161 * width_Body) / 2.0f - (t2 * t3 * t9 * t173 * width_Body) / 2.0f - height_Body * t2 * t4 * t31 * t55 * t111 * 6.0f;
        float A33 = t133 - t148 - t160 + t183 + (t6 * t8 * width_Body * (t106 - t144)) / 2.0f - (t3 * t4 * t6 * t163 * width_Body) / 2.0f + (t2 * t3 * t9 * width_Body * (t106 - t144)) / 2.0f - height_Body * t2 * t4 * t31 * t55 * t105 * 6.0f;
        float A34 = t125 + t129 + t130 + t139 - (t3 * t4 * t6 * t147 * width_Body) / 2.0f;
        float A35 = t137 - t154 + t157 - t164 + t182 - t47 * (t107 - t145) - t58 * (t107 - t145) - (t3 * t4 * t6 * t165 * width_Body) / 2.0f;
        float A36 = t148 + t151 + t160 + t183 + t47 * t171 + t58 * t171 - (t3 * t4 * t6 * t166 * width_Body) / 2.0f - height_Body * t3 * t4 * t9 * t24 * t31 * t56 * width_Body * 3.0f;
        float A37 = t14 * t61 + t15 * t61 + t17 * t61 + t109 * t185 + t146 * t183 - t134 * t142 * t170 * 3.0f - (t2 * t8 * t14 * width_Body) / 2.0f - (t2 * t8 * t15 * width_Body) / 2.0f - (t6 * t8 * t187 * width_Body) / 2.0f - dalpha_Body * dbeta_Body * t3 * t6 * width_Body - (t2 * t3 * t9 * t187 * width_Body) / 2.0f - (t3 * t4 * t6 * t188 * width_Body) / 2.0f - t2 * t3 * t81 * t143 * width_Body * 6.0f - height_Body * t2 * t4 * t31 * t55 * t169 * 6.0f - t4 * t6 * t17 * t19 * t26 * t56 * (3.0f / 2.0f) - t6 * t8 * t9 * t81 * t143 * width_Body * 6.0f + dalpha_Body * dbeta_Body * t2 * t8 * t9 * width_Body - dalpha_Body * dgamma_Body * t2 * t3 * t4 * width_Body + dbeta_Body * dgamma_Body * t4 * t6 * t8 * width_Body;

        return new Matrix(new float[,]
            {
                {A11, A12, A13, A14, A15, A16, A17 },
                {A21, A22, A13, A24, A25, A26, A27 },
                {A31, A32, A33, A34, A35, A36, A37 },
            });
    }

    public static Matrix FFD_Coeffs_Ddr_Arm_Bottom(float dr_Beta_Hand, float dr_Alpha_Hand, float g, float length_Hand, float m_Hand, float r_Alpha_Hand, float r_Beta_Hand, float r_Tau_Beta_Shoulder, float r_Tau_Alpha_Shoulder)
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
        float t21 = (g * length_Hand * m_Hand * t2 * t3) / 2.0f;
        float t22 = (g * length_Hand * m_Hand * t4 * t5) / 2.0f;
        float t16 = r_Tau_Beta_Shoulder * t9;
        float t17 = r_Tau_Beta_Shoulder * t11;
        float t18 = t9 * t10 * 3.0f;
        float t19 = t10 * t15 * 3.0f;
        float t20 = t10 * t11 * 3.0f;
        float t23 = -t22;
        float t24 = t9 * t12 * t15 * 3.0f;
        float t25 = t11 * t12 * t15 * 3.0f;
        float t26 = (t3 * t5 * t6 * t15) / 4.0f;
        float t28 = (dr_Beta_Hand * dr_Alpha_Hand * t3 * t5 * t9 * t15) / 2.0f;
        float t29 = (dr_Beta_Hand * dr_Alpha_Hand * t3 * t5 * t11 * t15) / 2.0f;
        float t31 = (t3 * t5 * t7 * t9 * t15) / 4.0f;
        float t34 = (t3 * t5 * t7 * t11 * t15) / 4.0f;
        float t27 = -t26;
        float t30 = t9 * t26;
        float t32 = -t28;
        float t33 = t11 * t26;
        float t35 = -t29;
        float t36 = t18 + t20 + 1.0f;
        float t39 = t15 + t19 + t24 + t25;
        float t37 = 1.0f / t36;
        float t40 = 1.0f / t39;
        float A11 = t8 * t10 * t40 * -1.2e+1f;
        float t41 = r_Tau_Alpha_Shoulder + t21 + t32 + t35;
        float t49 = t16 + t17 + t23 + t27 + t30 + t31 + t33 + t34;
        float t38 = t2 * t4 * t10 * t14 * t37 * 1.2e+1f;
        float t42 = t2 * t3 * t5 * t8 * t40 * 1.2e+1f;
        float t43 = t3 * t4 * t5 * t8 * t40 * 1.2e+1f;
        float t46 = t2 * t4 * t8 * t12 * t40 * 1.2e+1f;
        float t44 = -t42;
        float A12 = t44;
        float t45 = -t43;
        float A13 = t45;
        float A14 = length_Hand * t5 * t6 + length_Hand * t3 * t40 * t49 * 1.2e+1f;
        float A21 = t44;
        float A22 = t8 * t9 * t12 * t40 * -1.2e+1f - t10 * t11 * t14 * t37 * 1.2e+1f;
        float t47 = -t46;
        float t48 = t38 + t47;
        float A23 = t48;
        float A24 = -length_Hand * t2 * t3 * t6 - length_Hand * t2 * t3 * t7 + dr_Beta_Hand * dr_Alpha_Hand * length_Hand * t4 * t5 * 2.0f + length_Hand * t2 * t5 * t40 * t49 * 1.2e+1f + t3 * t4 * t13 * t14 * t37 * t41 * 1.2e+1f;
        float A31 = t45;
        float A32 = t48;
        float A33 = t9 * t10 * t14 * t37 * -1.2e+1f - t8 * t11 * t12 * t40 * 1.2e+1f;
        float A34 = -length_Hand * t3 * t4 * t6 - length_Hand * t3 * t4 * t7 - dr_Beta_Hand * dr_Alpha_Hand * length_Hand * t2 * t5 * 2.0f + length_Hand * t4 * t5 * t40 * t49 * 1.2e+1f - t2 * t3 * t13 * t14 * t37 * t41 * 1.2e+1f;

        return new Matrix(new float[,]
        {
            { A11, A12, A13, 0, 0, 0, A14 },
            { A21, A22, A23, 0, 0, 0, A24 },
            { A31, A32, A33, 0, 0, 0, A34 }
        });
    }

    public static Matrix FFD_Coeffs_Ddr_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float depth_Body, float dgamma_Body, float g, float gamma_Body, float height_Body, float l_Alpha_Hand, float l_Tau_Beta_Shoulder, float l_Tau_Alpha_Shoulder, float m_Body, float r_Alpha_Hand, float r_Tau_Beta_Shoulder, float r_Tau_Alpha_Shoulder, float width_Body)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Cos(l_Alpha_Hand);
        float t6 = Mathf.Sin(alpha_Body);
        float t7 = Mathf.Cos(r_Alpha_Hand);
        float t8 = Mathf.Sin(beta_Body);
        float t9 = Mathf.Sin(gamma_Body);
        float t10 = Mathf.Sin(l_Alpha_Hand);
        float t11 = Mathf.Sin(r_Alpha_Hand);
        float t12 = g * m_Body;
        float t13 = l_Tau_Alpha_Shoulder + r_Tau_Alpha_Shoulder;
        float t14 = Mathf.Pow(dalpha_Body, 2f);
        float t15 = Mathf.Pow(dbeta_Body, 2f);
        float t16 = Mathf.Pow(depth_Body, 2f);
        float t17 = Mathf.Pow(dgamma_Body, 2f);
        float t18 = Mathf.Pow(height_Body, 2f);
        float t19 = Mathf.Pow(height_Body, 3f);
        float t21 = Mathf.Pow(width_Body, 2f);
        float t31 = 1.0f / m_Body;
        float t20 = Mathf.Pow(t18, 2f);
        float t22 = Mathf.Pow(t2, 2f);
        float t23 = Mathf.Pow(t4, 2f);
        float t24 = Mathf.Pow(t6, 2f);
        float t25 = Mathf.Pow(t8, 2f);
        float t26 = Mathf.Pow(t9, 2f);
        float t27 = l_Tau_Beta_Shoulder * t5;
        float t28 = r_Tau_Beta_Shoulder * t7;
        float t29 = l_Tau_Beta_Shoulder * t10;
        float t30 = r_Tau_Beta_Shoulder * t11;
        float t32 = -t12;
        float t33 = m_Body * t16;
        float t34 = m_Body * t21;
        float t35 = t9 * t13;
        float t36 = t16 * t18;
        float t37 = t16 + t18;
        float t38 = t16 * t21;
        float t39 = t18 * t21;
        float t40 = t18 + t21;
        float t41 = dalpha_Body * dgamma_Body * height_Body * t6 * t9;
        float t44 = (t2 * t3 * width_Body) / 2.0f;
        float t45 = (t2 * t8 * width_Body) / 2.0f;
        float t46 = (t3 * t6 * width_Body) / 2.0f;
        float t47 = (t6 * t8 * width_Body) / 2.0f;
        float t48 = dalpha_Body * dgamma_Body * height_Body * m_Body * t2 * t9;
        float t51 = height_Body * t2 * t4 * t14 * 2.0f;
        float t52 = height_Body * t2 * t4 * t17 * 2.0f;
        float t57 = (height_Body * t2 * t4 * t12) / 2.0f;
        float t59 = (height_Body * t6 * t9 * t12) / 2.0f;
        float t64 = (height_Body * t2 * t4 * t14) / 2.0f;
        float t65 = (height_Body * t2 * t4 * t17) / 2.0f;
        float t67 = t2 * t8 * t9 * width_Body * (-1.0f / 2.0f);
        float t68 = t3 * t6 * t9 * width_Body * (-1.0f / 2.0f);
        float t77 = (height_Body * m_Body * t4 * t6 * t14) / 2.0f;
        float t78 = (height_Body * m_Body * t4 * t6 * t17) / 2.0f;
        float t85 = (m_Body * t4 * t9 * t17 * t18) / 4.0f;
        float t42 = t41 * 4.0f;
        float t43 = t18 * t23 * 3.0f;
        float t49 = t27 + t28;
        float t50 = t29 + t30;
        float t53 = -t41;
        float t55 = 1.0f / t37;
        float t56 = 1.0f / t40;
        float t58 = t9 * t44;
        float t60 = t9 * t45;
        float t61 = t9 * t46;
        float t62 = t9 * t47;
        float t63 = t33 + t34;
        float t66 = -t57;
        float t70 = t2 * t6 * t20 * t23;
        float t72 = t2 * t6 * t20 * t26;
        float t73 = t20 * t22 * t23 * 3.0f;
        float t74 = t20 * t22 * t26 * 3.0f;
        float t75 = t20 * t23 * t24 * 3.0f;
        float t76 = t20 * t24 * t26 * 3.0f;
        float t83 = t2 * t6 * t26 * t36;
        float t84 = t2 * t6 * t23 * t39;
        float t86 = t22 * t26 * t36 * 3.0f;
        float t87 = t22 * t23 * t39 * 3.0f;
        float t89 = t24 * t26 * t36 * 3.0f;
        float t90 = t23 * t24 * t39 * 3.0f;
        float t93 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t18 * t22) / 2.0f;
        float t94 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t18 * t24) / 2.0f;
        float t95 = (m_Body * t4 * t9 * t14 * t18 * t22) / 4.0f;
        float t96 = t22 * t85;
        float t97 = (m_Body * t4 * t9 * t14 * t18 * t24) / 4.0f;
        float t98 = t24 * t85;
        float t101 = m_Body * t4 * t9 * t17 * t18 * t22 * (-1.0f / 4.0f);
        float t103 = m_Body * t4 * t9 * t17 * t18 * t24 * (-1.0f / 4.0f);
        float t111 = t45 + t68;
        float t112 = t46 + t67;
        float t123 = t20 + t36 + t38 + t39;
        float t146 = t32 + t48 + t77 + t78;
        float t54 = -t42;
        float t69 = t2 * t49;
        float t71 = t6 * t50;
        float t79 = t4 * t6 * t49;
        float t80 = t2 * t4 * t50;
        float t81 = 1.0f / t63;
        float t82 = -t72;
        float t88 = t40 + t43;
        float t92 = -t83;
        float t99 = height_Body * t4 * t31 * t56 * 6.0f;
        float t100 = -t95;
        float t102 = -t97;
        float t104 = t44 + t62;
        float t105 = t47 + t58;
        float t106 = height_Body * t2 * t4 * t31 * t55 * 6.0f;
        float t107 = height_Body * t4 * t6 * t31 * t55 * 6.0f;
        float t108 = height_Body * t2 * t9 * t31 * t56 * 6.0f;
        float t109 = height_Body * t6 * t9 * t31 * t56 * 6.0f;
        float t110 = t3 * t9 * t31 * t56 * width_Body * 6.0f;
        float t116 = t4 * t9 * t17 * t18 * t56 * 3.0f;
        float t117 = t2 * t3 * t4 * t31 * t56 * width_Body * 6.0f;
        float t118 = t3 * t4 * t6 * t31 * t56 * width_Body * 6.0f;
        float t122 = height_Body * t3 * t4 * t9 * t31 * t56 * width_Body * 3.0f;
        float t124 = t2 * t4 * t9 * t18 * t31 * t56 * 3.0f;
        float t125 = t4 * t6 * t9 * t18 * t31 * t56 * 3.0f;
        float t126 = height_Body * t2 * t3 * t23 * t31 * t56 * width_Body * 3.0f;
        float t127 = height_Body * t2 * t3 * t26 * t31 * t56 * width_Body * 3.0f;
        float t128 = height_Body * t3 * t6 * t23 * t31 * t56 * width_Body * 3.0f;
        float t129 = height_Body * t3 * t6 * t26 * t31 * t56 * width_Body * 3.0f;
        float t134 = 1.0f / t123;
        float t137 = height_Body * t2 * t3 * t4 * t6 * t9 * t31 * t56 * width_Body * -3.0f;
        float t142 = t53 + t64 + t65;
        float t145 = t31 * t55 * t111 * 1.2e+1f;
        float t169 = t13 + t66 + t93 + t94;
        float t177 = height_Body * t2 * t4 * t31 * t55 * t146 * -6.0f;
        float t179 = t73 + t76 + t87 + t89 + t123;
        float t180 = t74 + t75 + t86 + t90 + t123;
        float t91 = -t79;
        float t113 = -t106;
        float t114 = -t107;
        float t115 = -t110;
        float t119 = -t116;
        float t120 = -t117;
        float t121 = -t118;
        float t130 = t2 * t3 * t4 * t8 * t21 * t81 * 3.0f;
        float t131 = t3 * t4 * t6 * t8 * t21 * t81 * 3.0f;
        float t132 = t21 * t23 * t25 * t81 * 3.0f;
        float t133 = t24 * t122;
        float t135 = t2 * t6 * t122;
        float t136 = t22 * t122;
        float t138 = t2 * t4 * t9 * t21 * t25 * t81 * 3.0f;
        float t139 = t4 * t6 * t9 * t21 * t25 * t81 * 3.0f;
        float t140 = t51 + t52 + t54;
        float t141 = t31 * t56 * t88;
        float t144 = t31 * t55 * t105 * 1.2e+1f;
        float t147 = t99 + t110;
        float t148 = t2 * t3 * t81 * t104 * width_Body * 6.0f;
        float t149 = t3 * t6 * t81 * t104 * width_Body * 6.0f;
        float t150 = t4 * t8 * t81 * t104 * width_Body * 6.0f;
        float t151 = t105 * t106;
        float t152 = t105 * t107;
        float t154 = t2 * t3 * t81 * t112 * width_Body * 6.0f;
        float t155 = t3 * t6 * t81 * t112 * width_Body * 6.0f;
        float t156 = t4 * t8 * t81 * t112 * width_Body * 6.0f;
        float t157 = t106 * t111;
        float t158 = t107 * t111;
        float t159 = t2 * t8 * t9 * t81 * t104 * width_Body * 6.0f;
        float t160 = t6 * t8 * t9 * t81 * t104 * width_Body * 6.0f;
        float t161 = t108 + t117;
        float t162 = t2 * t8 * t9 * t81 * t112 * width_Body * 6.0f;
        float t163 = t109 + t118;
        float t164 = t6 * t8 * t9 * t81 * t112 * width_Body * 6.0f;
        float t170 = t70 + t82 + t84 + t92;
        float t173 = t107 + t145;
        float t174 = t109 * t146;
        float t176 = t106 * t146;
        float t178 = t31 * t55 * t169 * 1.2e+1f;
        float t183 = t31 * t134 * t179;
        float t184 = t31 * t134 * t180;
        float t185 = t59 + t69 + t71 + t85 + t100 + t101 + t102 + t103;
        float t143 = t35 + t80 + t91;
        float t153 = t99 + t115;
        float A11 = -t122 + t132 + t141 - (t3 * t9 * t153 * width_Body) / 2.0f;
        float A12 = t124 + t126 - t156 - (t3 * t9 * t161 * width_Body) / 2.0f;
        float A13 = t125 + t128 + t150 - (t3 * t9 * t163 * width_Body) / 2.0f;
        float A14 = t122 - t132 + t141 - (t3 * t9 * t147 * width_Body) / 2.0f;
        float t165 = t108 + t120;
        float A15 = t124 - t126 + t156 - (t3 * t9 * t165 * width_Body) / 2.0f;
        float t166 = t109 + t121;
        float A16 = t125 - t128 - t150 - (t3 * t9 * t166 * width_Body) / 2.0f;
        float t167 = height_Body * t4 * t6 * t55 * t140 * (3.0f / 2.0f);
        float t168 = height_Body * t2 * t9 * t56 * t140 * (3.0f / 2.0f);
        float t171 = t106 + t144;
        float t172 = t113 + t144;
        float t175 = t114 + t145;
        float t181 = t31 * t134 * t170 * 3.0f;
        float t186 = t31 * t56 * t185 * 1.2e+1f;
        float t182 = -t181;
        float t187 = t167 + t177 + t178;
        float t188 = t119 + t168 + t174 + t186;
        float A17 = t125 * t146 + t99 * t185 - (t3 * t4 * t15 * width_Body) / 2.0f - (t3 * t4 * t17 * width_Body) / 2.0f - (t3 * t9 * t188 * width_Body) / 2.0f + dbeta_Body * dgamma_Body * t8 * t9 * width_Body - (height_Body * t9 * t17 * t56 * t88) / 2.0f + t4 * t8 * t81 * t143 * width_Body * 6.0f + t2 * t4 * t9 * t18 * t56 * t140 * (3.0f / 4.0f);
        float A21 = t124 - t127 - t131 + t138 + t4 * t44 * t153;
        float A22 = t136 + t155 + t158 - t162 + t184 + t45 * t173 + t68 * t173 + t4 * t44 * t161;
        float A23 = t135 - t149 + t152 + t159 + t182 - t45 * (t106 - t144) - t68 * (t106 - t144) + t4 * t44 * t163;
        float A24 = t124 + t127 + t131 - t138 + t4 * t44 * t147;
        float A25 = -t155 + t162 + t184 - t61 * (t107 - t145) + t4 * t44 * t165 + (t2 * t8 * width_Body * (t107 - t145)) / 2.0f - height_Body * t4 * t6 * t31 * t55 * t111 * 6.0f - height_Body * t3 * t4 * t9 * t22 * t31 * t56 * width_Body * 3.0f;
        float A26 = t137 + t149 - t159 + t182 + t61 * t171 + t4 * t44 * t166 - (t2 * t8 * t171 * width_Body) / 2.0f - height_Body * t4 * t6 * t31 * t55 * t105 * 6.0f;
        float A27 = t45 * t187 + t68 * t187 + t107 * t169 + t108 * t185 + t4 * t44 * t188 + t134 * t142 * t180 - t31 * t134 * t146 * t170 * 3.0f - (t6 * t8 * t14 * width_Body) / 2.0f - (t6 * t8 * t15 * width_Body) / 2.0f + dalpha_Body * dbeta_Body * t2 * t3 * width_Body - (t2 * t3 * t9 * t14 * width_Body) / 2.0f - (t2 * t3 * t9 * t15 * width_Body) / 2.0f - (t2 * t3 * t9 * t17 * width_Body) / 2.0f - t3 * t6 * t81 * t143 * width_Body * 6.0f - t2 * t4 * t17 * t19 * t26 * t56 * (3.0f / 2.0f) + t2 * t8 * t9 * t81 * t143 * width_Body * 6.0f + dalpha_Body * dbeta_Body * t6 * t8 * t9 * width_Body - dalpha_Body * dgamma_Body * t3 * t4 * t6 * width_Body - dbeta_Body * dgamma_Body * t2 * t4 * t8 * width_Body;
        float A31 = t125 - t129 + t130 + t139 + t4 * t46 * t153;
        float A32 = t135 - t154 - t164 + t182 + t47 * t173 + t58 * t173 + t4 * t46 * t161 - height_Body * t2 * t4 * t31 * t55 * t111 * 6.0f;
        float A33 = t133 + t148 + t160 + t183 - t47 * (t106 - t144) - t58 * (t106 - t144) + t4 * t46 * t163 - height_Body * t2 * t4 * t31 * t55 * t105 * 6.0f;
        float A34 = t125 + t129 - t130 - t139 + t4 * t46 * t147;
        float A35 = t137 + t154 + t157 + t164 + t182 + t4 * t46 * t165 + (t6 * t8 * width_Body * (t107 - t145)) / 2.0f + (t2 * t3 * t9 * width_Body * (t107 - t145)) / 2.0f;
        float A36 = -t148 + t151 - t160 + t183 + t4 * t46 * t166 - (t6 * t8 * t171 * width_Body) / 2.0f - (t2 * t3 * t9 * t171 * width_Body) / 2.0f - height_Body * t3 * t4 * t9 * t24 * t31 * t56 * width_Body * 3.0f;
        float A37 = t14 * t45 + t15 * t45 + t14 * t68 + t15 * t68 + t17 * t68 + t47 * t187 + t58 * t187 + t109 * t185 + t146 * t183 + t4 * t46 * t188 - t134 * t142 * t170 * 3.0f + dalpha_Body * dbeta_Body * t3 * t6 * width_Body + t2 * t3 * t81 * t143 * width_Body * 6.0f - height_Body * t2 * t4 * t31 * t55 * t169 * 6.0f - t4 * t6 * t17 * t19 * t26 * t56 * (3.0f / 2.0f) + t6 * t8 * t9 * t81 * t143 * width_Body * 6.0f - dalpha_Body * dbeta_Body * t2 * t8 * t9 * width_Body + dalpha_Body * dgamma_Body * t2 * t3 * t4 * width_Body - dbeta_Body * dgamma_Body * t4 * t6 * t8 * width_Body;

        return new Matrix(new float[,]
            {
                {A11, A12, A13, A14, A15, A16, A17 },
                {A21, A22, A13, A24, A25, A26, A27 },
                {A31, A32, A33, A34, A35, A36, A37 },
            });
    }

    public static float[] FFD_Dds_Arm_L(float dl_Beta_Hand, float dl_Alpha_Hand, float g, float l_Alpha_Hand, float l_Beta_Hand, float l_F_X, float l_F_Y, float l_F_Z, float l_Tau_Beta_Shoulder, float l_Tau_Alpha_Shoulder, float length_Hand, float m_Hand)
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
        float ddl_Alpha_Hand = ((l_Tau_Alpha_Shoulder + l_F_Z * length_Hand * t2 * t3 - l_F_Y * length_Hand * t3 * t4 + (g * length_Hand * m_Hand * t2 * t3) / 2.0f - (dl_Beta_Hand * dl_Alpha_Hand * m_Hand * t3 * t5 * t8 * t9) / 2.0f - (dl_Beta_Hand * dl_Alpha_Hand * m_Hand * t3 * t5 * t8 * t11) / 2.0f) * -1.2e+1f) / (m_Hand * t8 * (t9 * t10 * 3.0f + t10 * t11 * 3.0f + 1.0f));
        float t12 = Mathf.Pow(t5, 2f);
        float ddl_Beta_Hand = ((l_Tau_Beta_Shoulder * t9 + l_Tau_Beta_Shoulder * t11 + l_F_X * length_Hand * t3 - l_F_Y * length_Hand * t2 * t5 - l_F_Z * length_Hand * t4 * t5 - (g * length_Hand * m_Hand * t4 * t5) / 2.0f - (m_Hand * t3 * t5 * t6 * t8) / 4.0f + (m_Hand * t3 * t5 * t6 * t8 * t9) / 4.0f + (m_Hand * t3 * t5 * t7 * t8 * t9) / 4.0f + (m_Hand * t3 * t5 * t6 * t8 * t11) / 4.0f + (m_Hand * t3 * t5 * t7 * t8 * t11) / 4.0f) * -1.2e+1f) / (m_Hand * t8 + m_Hand * t8 * t10 * 3.0f + m_Hand * t8 * t9 * t12 * 3.0f + m_Hand * t8 * t11 * t12 * 3.0f);

        return new float[] { ddl_Alpha_Hand, ddl_Beta_Hand };

    }

    public static float[] FFD_Dds_Arm_R(float dr_Beta_Hand, float dr_Alpha_Hand, float g, float length_Hand, float m_Hand, float r_Alpha_Hand, float r_Beta_Hand, float r_F_X, float r_F_Y, float r_F_Z, float r_Tau_Beta_Shoulder, float r_Tau_Alpha_Shoulder)
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
        float ddr_Alpha_Hand = ((r_Tau_Alpha_Shoulder + length_Hand * r_F_Z * t2 * t3 - length_Hand * r_F_Y * t3 * t4 + (g * length_Hand * m_Hand * t2 * t3) / 2.0f - (dr_Beta_Hand * dr_Alpha_Hand * m_Hand * t3 * t5 * t8 * t9) / 2.0f - (dr_Beta_Hand * dr_Alpha_Hand * m_Hand * t3 * t5 * t8 * t11) / 2.0f) * -1.2e+1f) / (m_Hand * t8 * (t9 * t10 * 3.0f + t10 * t11 * 3.0f + 1.0f));
        float t12 = Mathf.Pow(t5, 2f);
        float ddr_Beta_Hand = ((r_Tau_Beta_Shoulder * t9 + r_Tau_Beta_Shoulder * t11 - length_Hand * r_F_X * t3 - length_Hand * r_F_Y * t2 * t5 - length_Hand * r_F_Z * t4 * t5 - (g * length_Hand * m_Hand * t4 * t5) / 2.0f - (m_Hand * t3 * t5 * t6 * t8) / 4.0f + (m_Hand * t3 * t5 * t6 * t8 * t9) / 4.0f + (m_Hand * t3 * t5 * t7 * t8 * t9) / 4.0f + (m_Hand * t3 * t5 * t6 * t8 * t11) / 4.0f + (m_Hand * t3 * t5 * t7 * t8 * t11) / 4.0f) * -1.2e+1f) / (m_Hand * t8 + m_Hand * t8 * t10 * 3.0f + m_Hand * t8 * t9 * t12 * 3.0f + m_Hand * t8 * t11 * t12 * 3.0f);

        return new float[] { ddr_Alpha_Hand, ddr_Beta_Hand };

    }

    public static float[] FFD_Dds_Body(float alpha_Body, float beta_Body, float dalpha_Body, float depth_Body, float dgamma_Body, float g, float gamma_Body, float height_Body, float l_Alpha_Hand, float l_F_X, float l_F_Y, float l_F_Z, float l_Tau_Beta_Shoulder, float l_Tau_Alpha_Shoulder, float m_Body, float r_Alpha_Hand, float r_F_X, float r_F_Y, float r_F_Z, float r_Tau_Beta_Shoulder, float r_Tau_Alpha_Shoulder, float width_Body)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Cos(l_Alpha_Hand);
        float t6 = Mathf.Sin(alpha_Body);
        float t7 = Mathf.Cos(r_Alpha_Hand);
        float t8 = Mathf.Sin(beta_Body);
        float t9 = Mathf.Sin(gamma_Body);
        float t10 = Mathf.Sin(l_Alpha_Hand);
        float t11 = Mathf.Sin(r_Alpha_Hand);
        float t12 = g * m_Body;
        float t13 = Mathf.Pow(dalpha_Body, 2f);
        float t14 = Mathf.Pow(depth_Body, 2f);
        float t15 = Mathf.Pow(dgamma_Body, 2f);
        float t16 = Mathf.Pow(height_Body, 2f);
        float t18 = Mathf.Pow(width_Body, 2f);
        float t27 = 1.0f / m_Body;
        float t17 = Mathf.Pow(t16, 2f);
        float t19 = Mathf.Pow(t2, 2f);
        float t20 = Mathf.Pow(t4, 2f);
        float t21 = Mathf.Pow(t6, 2f);
        float t22 = Mathf.Pow(t9, 2f);
        float t23 = l_Tau_Beta_Shoulder * t5;
        float t24 = r_Tau_Beta_Shoulder * t7;
        float t25 = l_Tau_Beta_Shoulder * t10;
        float t26 = r_Tau_Beta_Shoulder * t11;
        float t28 = -t12;
        float t29 = t14 * t16;
        float t30 = t14 + t16;
        float t31 = t14 * t18;
        float t32 = t16 * t18;
        float t33 = t16 + t18;
        float t34 = dalpha_Body * dgamma_Body * height_Body * t6 * t9 * 4.0f;
        float t35 = (t2 * t3 * width_Body) / 2.0f;
        float t36 = (t2 * t8 * width_Body) / 2.0f;
        float t37 = (t3 * t6 * width_Body) / 2.0f;
        float t38 = (t6 * t8 * width_Body) / 2.0f;
        float t39 = dalpha_Body * dgamma_Body * height_Body * m_Body * t2 * t9;
        float t42 = height_Body * t2 * t4 * t13 * 2.0f;
        float t43 = height_Body * t2 * t4 * t15 * 2.0f;
        float t47 = (l_F_X * t3 * t9 * width_Body) / 2.0f;
        float t48 = (r_F_X * t3 * t9 * width_Body) / 2.0f;
        float t49 = (height_Body * t2 * t4 * t12) / 2.0f;
        float t51 = (height_Body * t6 * t9 * t12) / 2.0f;
        float t55 = (height_Body * m_Body * t9 * t15) / 2.0f;
        float t58 = t2 * t8 * t9 * width_Body * (-1.0f / 2.0f);
        float t59 = t3 * t6 * t9 * width_Body * (-1.0f / 2.0f);
        float t69 = (height_Body * m_Body * t4 * t6 * t13) / 2.0f;
        float t70 = (height_Body * m_Body * t4 * t6 * t15) / 2.0f;
        float t71 = l_F_Y * t2 * t3 * t4 * width_Body * (-1.0f / 2.0f);
        float t72 = l_F_Z * t3 * t4 * t6 * width_Body * (-1.0f / 2.0f);
        float t77 = (m_Body * t4 * t9 * t15 * t16) / 4.0f;
        float t40 = t23 + t24;
        float t41 = t25 + t26;
        float t44 = -t34;
        float t45 = 1.0f / t30;
        float t46 = 1.0f / t33;
        float t50 = t9 * t35;
        float t52 = t9 * t36;
        float t53 = t9 * t37;
        float t54 = t9 * t38;
        float t56 = -t48;
        float t57 = -t49;
        float t60 = l_F_Y * t4 * t35;
        float t61 = r_F_Y * t4 * t35;
        float t62 = l_F_Z * t4 * t37;
        float t63 = r_F_Z * t4 * t37;
        float t65 = -t55;
        float t66 = t2 * t6 * t17 * t20;
        float t68 = t2 * t6 * t17 * t22;
        float t75 = t2 * t6 * t22 * t29;
        float t76 = t2 * t6 * t20 * t32;
        float t79 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t16 * t19) / 2.0f;
        float t80 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t16 * t21) / 2.0f;
        float t81 = (m_Body * t4 * t9 * t13 * t16 * t19) / 4.0f;
        float t82 = t19 * t77;
        float t83 = (m_Body * t4 * t9 * t13 * t16 * t21) / 4.0f;
        float t84 = t21 * t77;
        float t86 = m_Body * t4 * t9 * t15 * t16 * t19 * (-1.0f / 4.0f);
        float t88 = m_Body * t4 * t9 * t15 * t16 * t21 * (-1.0f / 4.0f);
        float t91 = t36 + t59;
        float t92 = t37 + t58;
        float t97 = t17 + t29 + t31 + t32;
        float t104 = l_F_Z + r_F_Z + t28 + t39 + t69 + t70;
        float t64 = t2 * t40;
        float t67 = t6 * t41;
        float t73 = -t68;
        float t74 = l_F_X + r_F_X + t65;
        float t78 = -t75;
        float t85 = -t81;
        float t87 = -t83;
        float t89 = t35 + t54;
        float t90 = t38 + t50;
        float t95 = l_F_Y * t91;
        float t96 = r_F_Y * t91;
        float t100 = 1.0f / t97;
        float t101 = t42 + t43 + t44;
        float t93 = l_F_Z * t90;
        float t94 = r_F_Z * t90;
        float t99 = -t95;
        float t102 = (m_Body * t101) / 4.0f;
        float t105 = t66 + t73 + t76 + t78;
        float t107 = t47 + t51 + t56 + t61 + t63 + t64 + t67 + t71 + t72 + t77 + t85 + t86 + t87 + t88;
        float t98 = -t93;
        float t103 = l_F_Y + r_F_Y + t102;
        float t106 = l_Tau_Alpha_Shoulder + r_Tau_Alpha_Shoulder + t57 + t79 + t80 + t94 + t96 + t98 + t99;
        float ddalpha_Body = t27 * t45 * t106 * 1.2e+1f - height_Body * t2 * t4 * t27 * t45 * t104 * 6.0f + height_Body * t4 * t6 * t27 * t45 * t103 * 6.0f;
        float ddbeta_Body = ((t9 * (l_Tau_Alpha_Shoulder + r_Tau_Alpha_Shoulder) - l_F_Z * t89 + l_F_Y * t92 + r_F_Z * t89 - r_F_Y * t92 + t2 * t4 * t41 - t4 * t6 * t40 - (l_F_X * t4 * t8 * width_Body) / 2.0f + (r_F_X * t4 * t8 * width_Body) / 2.0f) * -1.2e+1f) / (m_Body * t14 + m_Body * t18);
        float ddgamma_Body = t27 * t46 * t107 * 1.2e+1f + height_Body * t4 * t27 * t46 * t74 * 6.0f + height_Body * t2 * t9 * t27 * t46 * t103 * 6.0f + height_Body * t6 * t9 * t27 * t46 * t104 * 6.0f;
        float ddx_Head = t27 * t46 * t74 * (t33 + t16 * t20 * 3.0f) + height_Body * t4 * t27 * t46 * t107 * 6.0f + t2 * t4 * t9 * t16 * t27 * t46 * t103 * 3.0f + t4 * t6 * t9 * t16 * t27 * t46 * t104 * 3.0f;
        float ddy_Head = t27 * t100 * t104 * t105 * -3.0f + t27 * t100 * t103 * (t97 + t17 * t19 * t22 * 3.0f + t17 * t20 * t21 * 3.0f + t19 * t22 * t29 * 3.0f + t20 * t21 * t32 * 3.0f) + height_Body * t4 * t6 * t27 * t45 * t106 * 6.0f + height_Body * t2 * t9 * t27 * t46 * t107 * 6.0f + t2 * t4 * t9 * t16 * t27 * t46 * t74 * 3.0f;
        float ddz_Head = t27 * t100 * t103 * t105 * -3.0f + t27 * t100 * t104 * (t97 + t17 * t19 * t20 * 3.0f + t17 * t21 * t22 * 3.0f + t19 * t20 * t32 * 3.0f + t21 * t22 * t29 * 3.0f) - height_Body * t2 * t4 * t27 * t45 * t106 * 6.0f + height_Body * t6 * t9 * t27 * t46 * t107 * 6.0f + t4 * t6 * t9 * t16 * t27 * t46 * t74 * 3.0f;

        return new float[] { ddalpha_Body, ddbeta_Body, ddgamma_Body, ddx_Head, ddy_Head, ddz_Head };

    }

    public static Matrix FRD_Ddl_Arm_Bottom(float ddl_Beta_Hand, float ddl_Alpha_Hand, float dl_Beta_Hand, float dl_Alpha_Hand, float l_Alpha_Hand, float l_Beta_Hand, float length_Hand)
    {
        float t2 = Mathf.Cos(l_Alpha_Hand);
        float t3 = Mathf.Cos(l_Beta_Hand);
        float t4 = Mathf.Sin(l_Alpha_Hand);
        float t5 = Mathf.Sin(l_Beta_Hand);
        float t6 = Mathf.Pow(dl_Beta_Hand, 2f);
        float t7 = Mathf.Pow(dl_Alpha_Hand, 2f);
        return Matrix.MatrixTranspose(new Matrix(new float[,] {
            { ddl_Beta_Hand * length_Hand * t3 - length_Hand * t5 * t6, -ddl_Beta_Hand * length_Hand * t2 * t5 - ddl_Alpha_Hand * length_Hand * t3 * t4 - length_Hand * t2 * t3 * t6 - length_Hand * t2 * t3 * t7 + dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t4 * t5 * 2.0f, -ddl_Beta_Hand * length_Hand * t4 * t5 + ddl_Alpha_Hand * length_Hand * t2 * t3 - length_Hand * t3 * t4 * t6 - length_Hand * t3 * t4 * t7 - dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t2 * t5 * 2.0f }
        }));

    }

    public static Matrix FRD_Ddl_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float ddalpha_Body, float ddbeta_Body, float ddgamma_Body, float ddx_Head, float ddy_Head, float ddz_Head, float dgamma_Body, float gamma_Body, float width_Body)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        float t8 = Mathf.Pow(dalpha_Body, 2f);
        float t9 = Mathf.Pow(dbeta_Body, 2f);
        float t10 = Mathf.Pow(dgamma_Body, 2f);
        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { ddx_Head + (ddbeta_Body * t4 * t6 * width_Body) / 2.0f + (ddgamma_Body * t3 * t7 * width_Body) / 2.0f + (t3 * t4 * t9 * width_Body) / 2.0f + (t3 * t4 * t10 * width_Body) / 2.0f - dbeta_Body * dgamma_Body * t6 * t7 * width_Body, ddy_Head - (ddalpha_Body * t2 * t6 * width_Body) / 2.0f - (ddbeta_Body * t3 * t5 * width_Body) / 2.0f + (t5 * t6 * t8 * width_Body) / 2.0f + (t5 * t6 * t9 * width_Body) / 2.0f - dalpha_Body * dbeta_Body * t2 * t3 * width_Body + (ddalpha_Body * t3 * t5 * t7 * width_Body) / 2.0f + (ddbeta_Body * t2 * t6 * t7 * width_Body) / 2.0f - (ddgamma_Body * t2 * t3 * t4 * width_Body) / 2.0f + (t2 * t3 * t7 * t8 * width_Body) / 2.0f + (t2 * t3 * t7 * t9 * width_Body) / 2.0f + (t2 * t3 * t7 * t10 * width_Body) / 2.0f - dalpha_Body * dbeta_Body * t5 * t6 * t7 * width_Body + dalpha_Body * dgamma_Body * t3 * t4 * t5 * width_Body + dbeta_Body * dgamma_Body * t2 * t4 * t6 * width_Body, ddz_Head - (ddalpha_Body * t5 * t6 * width_Body) / 2.0f + (ddbeta_Body * t2 * t3 * width_Body) / 2.0f - (t2 * t6 * t8 * width_Body) / 2.0f - (t2 * t6 * t9 * width_Body) / 2.0f - dalpha_Body * dbeta_Body * t3 * t5 * width_Body - (ddalpha_Body * t2 * t3 * t7 * width_Body) / 2.0f + (ddbeta_Body * t5 * t6 * t7 * width_Body) / 2.0f - (ddgamma_Body * t3 * t4 * t5 * width_Body) / 2.0f + (t3 * t5 * t7 * t8 * width_Body) / 2.0f + (t3 * t5 * t7 * t9 * width_Body) / 2.0f + (t3 * t5 * t7 * t10 * width_Body) / 2.0f + dalpha_Body * dbeta_Body * t2 * t6 * t7 * width_Body - dalpha_Body * dgamma_Body * t2 * t3 * t4 * width_Body + dbeta_Body * dgamma_Body * t4 * t5 * t6 * width_Body }
        }));


    }

    public static Matrix FRD_Ddr_Arm_Bottom(float ddr_Beta_Hand, float ddr_Alpha_Hand, float dr_Beta_Hand, float dr_Alpha_Hand, float length_Hand, float r_Alpha_Hand, float r_Beta_Hand)
    {

        float t2 = Mathf.Cos(r_Alpha_Hand);
        float t3 = Mathf.Cos(r_Beta_Hand);
        float t4 = Mathf.Sin(r_Alpha_Hand);
        float t5 = Mathf.Sin(r_Beta_Hand);
        float t6 = Mathf.Pow(dr_Beta_Hand, 2f);
        float t7 = Mathf.Pow(dr_Alpha_Hand, 2f);

        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { -ddr_Beta_Hand * length_Hand * t3 + length_Hand * t5 * t6, -ddr_Beta_Hand * length_Hand * t2 * t5 - ddr_Alpha_Hand * length_Hand * t3 * t4 - length_Hand * t2 * t3 * t6 - length_Hand * t2 * t3 * t7 + dr_Beta_Hand * dr_Alpha_Hand * length_Hand * t4 * t5 * 2.0f, -ddr_Beta_Hand * length_Hand * t4 * t5 + ddr_Alpha_Hand * length_Hand * t2 * t3 - length_Hand * t3 * t4 * t6 - length_Hand * t3 * t4 * t7 - dr_Beta_Hand * dr_Alpha_Hand * length_Hand * t2 * t5 * 2.0f }
        }));

    }

    public static Matrix FRD_Ddr_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float ddalpha_Body, float ddbeta_Body, float ddgamma_Body, float ddx_Head, float ddy_Head, float ddz_Head, float dgamma_Body, float gamma_Body, float width_Body)

    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        float t8 = Mathf.Pow(dalpha_Body, 2f);
        float t9 = Mathf.Pow(dbeta_Body, 2f);
        float t10 = Mathf.Pow(dgamma_Body, 2f);

        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { ddx_Head - (ddbeta_Body * t4 * t6 * width_Body) / 2.0f - (ddgamma_Body * t3 * t7 * width_Body) / 2.0f - (t3 * t4 * t9 * width_Body) / 2.0f - (t3 * t4 * t10 * width_Body) / 2.0f + dbeta_Body * dgamma_Body * t6 * t7 * width_Body, ddy_Head + (ddalpha_Body * t2 * t6 * width_Body) / 2.0f + (ddbeta_Body * t3 * t5 * width_Body) / 2.0f - (t5 * t6 * t8 * width_Body) / 2.0f - (t5 * t6 * t9 * width_Body) / 2.0f + dalpha_Body * dbeta_Body * t2 * t3 * width_Body - (ddalpha_Body * t3 * t5 * t7 * width_Body) / 2.0f - (ddbeta_Body * t2 * t6 * t7 * width_Body) / 2.0f + (ddgamma_Body * t2 * t3 * t4 * width_Body) / 2.0f - (t2 * t3 * t7 * t8 * width_Body) / 2.0f - (t2 * t3 * t7 * t9 * width_Body) / 2.0f - (t2 * t3 * t7 * t10 * width_Body) / 2.0f + dalpha_Body * dbeta_Body * t5 * t6 * t7 * width_Body - dalpha_Body * dgamma_Body * t3 * t4 * t5 * width_Body - dbeta_Body * dgamma_Body * t2 * t4 * t6 * width_Body, ddz_Head + (ddalpha_Body * t5 * t6 * width_Body) / 2.0f - (ddbeta_Body * t2 * t3 * width_Body) / 2.0f + (t2 * t6 * t8 * width_Body) / 2.0f + (t2 * t6 * t9 * width_Body) / 2.0f + dalpha_Body * dbeta_Body * t3 * t5 * width_Body + (ddalpha_Body * t2 * t3 * t7 * width_Body) / 2.0f - (ddbeta_Body * t5 * t6 * t7 * width_Body) / 2.0f + (ddgamma_Body * t3 * t4 * t5 * width_Body) / 2.0f - (t3 * t5 * t7 * t8 * width_Body) / 2.0f - (t3 * t5 * t7 * t9 * width_Body) / 2.0f - (t3 * t5 * t7 * t10 * width_Body) / 2.0f - dalpha_Body * dbeta_Body * t2 * t6 * t7 * width_Body + dalpha_Body * dgamma_Body * t2 * t3 * t4 * width_Body - dbeta_Body * dgamma_Body * t4 * t5 * t6 * width_Body }
        }));
    }

    public static Matrix FRD_Dl_Arm_Bottom(float dl_Beta_Hand, float dl_Alpha_Hand, float l_Alpha_Hand, float l_Beta_Hand, float length_Hand)
    {
        float t2 = Mathf.Cos(l_Alpha_Hand);
        float t3 = Mathf.Cos(l_Beta_Hand);
        float t4 = Mathf.Sin(l_Alpha_Hand);
        float t5 = Mathf.Sin(l_Beta_Hand);

        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { dl_Beta_Hand * length_Hand * t3, -dl_Beta_Hand * length_Hand * t2 * t5 - dl_Alpha_Hand * length_Hand * t3 * t4, -dl_Beta_Hand * length_Hand * t4 * t5 + dl_Alpha_Hand * length_Hand * t2 * t3 }
        }));
    }
    public static Matrix FRD_Dr_Arm_Bottom(float dr_Beta_Hand, float dr_Alpha_Hand, float length_Hand, float r_Alpha_Hand, float r_Beta_Hand)

    {
        float t2 = Mathf.Cos(r_Alpha_Hand);
        float t3 = Mathf.Cos(r_Beta_Hand);
        float t4 = Mathf.Sin(r_Alpha_Hand);
        float t5 = Mathf.Sin(r_Beta_Hand);

        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { -dr_Beta_Hand * length_Hand * t3, -dr_Beta_Hand * length_Hand * t2 * t5 - dr_Alpha_Hand * length_Hand * t3 * t4, -dr_Beta_Hand * length_Hand * t4 * t5 + dr_Alpha_Hand * length_Hand * t2 * t3 }
        }));
    }

    public static Matrix FRD_L_Arm_Bottom(float l_Alpha_Hand, float l_Beta_Hand, float l_X_Fixed, float l_Y_Fixed, float l_Z_Fixed, float length_Hand)

    {
        float t2 = Mathf.Cos(l_Beta_Hand);

        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { l_X_Fixed + length_Hand * Mathf.Sin(l_Beta_Hand), l_Y_Fixed + length_Hand * t2 * Mathf.Cos(l_Alpha_Hand), l_Z_Fixed + length_Hand * t2 * Mathf.Sin(l_Alpha_Hand) }
        }));
    }

    public static Matrix FRD_L_Hip(float alpha_Body, float beta_Body, float gamma_Body, float height_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        float t8 = height_Body * t4;
        float t9 = (t3 * t7 * width_Body) / 2.0f;
        float t10 = -t9;
        float t11 = t8 + t10;

        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { x_Head - height_Body * t7 - (t3 * t4 * width_Body) / 2.0f, y_Head + t2 * t11 - (t5 * t6 * width_Body) / 2.0f, z_Head + t5 * t11 + (t2 * t6 * width_Body) / 2.0f }
        })); ;
    }

    public static Matrix FRD_L_Shoulder(float alpha_Body, float beta_Body, float gamma_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Sin(alpha_Body);
        float t5 = Mathf.Sin(beta_Body);
        float t6 = Mathf.Sin(gamma_Body);
        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { x_Head - (t3 * width_Body * Mathf.Cos(gamma_Body)) / 2.0f, y_Head - (t4 * t5 * width_Body) / 2.0f - (t2 * t3 * t6 * width_Body) / 2.0f, z_Head + (t2 * t5 * width_Body) / 2.0f - (t3 * t4 * t6 * width_Body) / 2.0f }
        }));
    }

    public static Matrix FRD_R_Arm_Bottom(float length_Hand, float r_Alpha_Hand, float r_Beta_Hand, float r_X_Fixed, float r_Y_Fixed, float r_Z_Fixed)
    {
        float t2 = Mathf.Cos(r_Beta_Hand);
        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { r_X_Fixed - length_Hand * Mathf.Sin(r_Beta_Hand), r_Y_Fixed + length_Hand * t2 * Mathf.Cos(r_Alpha_Hand), r_Z_Fixed + length_Hand * t2 * Mathf.Sin(r_Alpha_Hand) }
        }));
    }

    public static Matrix FRD_R_Hip(float alpha_Body, float beta_Body, float gamma_Body, float height_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        float t8 = height_Body * t4;
        float t9 = (t3 * t7 * width_Body) / 2.0f;
        float t10 = t8 + t9;
        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { x_Head - height_Body * t7 + (t3 * t4 * width_Body) / 2.0f, y_Head + t2 * t10 + (t5 * t6 * width_Body) / 2.0f, z_Head + t5 * t10 - (t2 * t6 * width_Body) / 2.0f }
        }));
    }

    public static Matrix FRD_R_Shoulder(float alpha_Body, float beta_Body, float gamma_Body, float width_Body, float x_Head, float y_Head, float z_Head)
    {
        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Sin(alpha_Body);
        float t5 = Mathf.Sin(beta_Body);
        float t6 = Mathf.Sin(gamma_Body);
        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { x_Head + (t3 * width_Body * Mathf.Cos(gamma_Body)) / 2.0f, y_Head + (t4 * t5 * width_Body) / 2.0f + (t2 * t3 * t6 * width_Body) / 2.0f, z_Head - (t2 * t5 * width_Body) / 2.0f + (t3 * t4 * t6 * width_Body) / 2.0f }
        }));
    }

    public static Vector3 convert_Matrix_To_Vector3(Matrix target)
    {
        return new Vector3(target[0, 0], target[2, 0], target[1, 0]);
    }

    public static Matrix convert_Vector3_To_Matrix(Vector3 target)
    {
        return new Matrix(new float[][] {
            new float[]{target.x },
            new float[]{target.y },
            new float[]{target.z }
        });
    }
}
