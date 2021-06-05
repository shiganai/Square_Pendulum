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
        float t18 = (g * length_Hand * m_Hand * t2) / 2.0f;
        float t16 = l_Tau_Beta_Shoulder * t11;
        float t17 = l_Tau_Beta_Shoulder * t9;
        float t19 = t9 * t10 * 3.0f;
        float t20 = t9 * t15 * 3.0f;
        float t21 = t9 * t12 * 3.0f;
        float t22 = t10 * t11 * t15 * 3.0f;
        float t23 = t11 * t12 * t15 * 3.0f;
        float t24 = (t2 * t4 * t7 * t15) / 4.0f;
        float t26 = (dl_Beta_Hand * dl_Alpha_Hand * t2 * t4 * t10 * t15) / 2.0f;
        float t27 = (dl_Beta_Hand * dl_Alpha_Hand * t2 * t4 * t12 * t15) / 2.0f;
        float t28 = (t2 * t4 * t6 * t10 * t15) / 4.0f;
        float t31 = (t2 * t4 * t6 * t12 * t15) / 4.0f;
        float t25 = -t24;
        float t29 = t10 * t24;
        float t30 = -t26;
        float t32 = t12 * t24;
        float t33 = -t27;
        float t34 = t19 + t21 + 1.0f;
        float t37 = t15 + t20 + t22 + t23;
        float t35 = 1.0f / t34;
        float t38 = 1.0f / t37;
        float A11 = t9 * t10 * t14 * t35 * -1.2e+1f - t8 * t11 * t12 * t38 * 1.2e+1f;
        float t39 = t16 + t17 + t30 + t33;
        float t45 = l_Tau_Alpha_Shoulder + t18 + t25 + t28 + t29 + t31 + t32;
        float t36 = t3 * t5 * t9 * t14 * t35 * 1.2e+1f;
        float t40 = t2 * t3 * t4 * t8 * t38 * 1.2e+1f;
        float t41 = t2 * t4 * t5 * t8 * t38 * 1.2e+1f;
        float t42 = t3 * t5 * t8 * t11 * t38 * 1.2e+1f;
        float t43 = -t42;
        float t44 = t36 + t43;
        float A12 = t44;
        float A13 = t41;
        float A14 = -length_Hand * t2 * t5 * t6 - length_Hand * t2 * t5 * t7 - dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t3 * t4 * 2.0f + length_Hand * t4 * t5 * t38 * t45 * 1.2e+1f - t2 * t3 * t13 * t14 * t35 * t39 * 1.2e+1f;
        float A21 = t44;
        float A22 = t8 * t10 * t11 * t38 * -1.2e+1f - t9 * t12 * t14 * t35 * 1.2e+1f;
        float A23 = t40;
        float A24 = -length_Hand * t2 * t3 * t6 - length_Hand * t2 * t3 * t7 + dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t4 * t5 * 2.0f + length_Hand * t3 * t4 * t38 * t45 * 1.2e+1f + t2 * t5 * t13 * t14 * t35 * t39 * 1.2e+1f;
        float A31 = t41;
        float A32 = t40;
        float A33 = t8 * t9 * t38 * -1.2e+1f;
        float A34 = -length_Hand * t4 * t7 - length_Hand * t2 * t38 * t45 * 1.2e+1f;

        return new Matrix(new float[,]
        {
            { 0, 0, 0, A11, A12, A13, A14 },
            { 0, 0, 0, A21, A22, A23, A24 },
            { 0, 0, 0, A31, A32, A33, A34 }
        });
    }

    public static Matrix FFD_Coeffs_Ddl_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float depth_Body, float dgamma_Body, float g, float gamma_Body, float height_Body, float l_Alpha_Hand, float l_Tau_Beta_Shoulder, float l_Tau_Alpha_Shoulder, float m_Body, float r_Beta_Hand, float r_Tau_Beta_Shoulder, float r_Tau_Alpha_Shoulder, float width_Body)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Cos(l_Alpha_Hand);
        float t6 = Mathf.Sin(alpha_Body);
        float t7 = Mathf.Cos(r_Beta_Hand);
        float t8 = Mathf.Sin(beta_Body);
        float t9 = Mathf.Sin(gamma_Body);
        float t10 = Mathf.Sin(l_Alpha_Hand);
        float t11 = Mathf.Sin(r_Beta_Hand);
        float t12 = g * m_Body;
        float t13 = Mathf.Pow(dalpha_Body, 2f);
        float t14 = Mathf.Pow(dbeta_Body, 2f);
        float t15 = Mathf.Pow(depth_Body, 2f);
        float t16 = Mathf.Pow(dgamma_Body, 2f);
        float t17 = Mathf.Pow(height_Body, 2f);
        float t18 = Mathf.Pow(height_Body, 3f);
        float t20 = Mathf.Pow(width_Body, 2f);
        float t30 = 1.0f / m_Body;
        float t19 = Mathf.Pow(t17, 2f);
        float t21 = Mathf.Pow(t2, 2f);
        float t22 = Mathf.Pow(t4, 2f);
        float t23 = Mathf.Pow(t6, 2f);
        float t24 = Mathf.Pow(t8, 2f);
        float t25 = Mathf.Pow(t9, 2f);
        float t26 = l_Tau_Beta_Shoulder * t5;
        float t27 = r_Tau_Alpha_Shoulder * t7;
        float t28 = l_Tau_Beta_Shoulder * t10;
        float t29 = r_Tau_Alpha_Shoulder * t11;
        float t31 = -t12;
        float t32 = m_Body * t15;
        float t33 = m_Body * t20;
        float t37 = t15 * t17;
        float t38 = t15 + t17;
        float t39 = t15 * t20;
        float t40 = t17 * t20;
        float t41 = t17 + t20;
        float t42 = dalpha_Body * dgamma_Body * height_Body * t6 * t9;
        float t45 = (t2 * t3 * width_Body) / 2.0f;
        float t46 = (t2 * t8 * width_Body) / 2.0f;
        float t47 = (t3 * t6 * width_Body) / 2.0f;
        float t48 = (t6 * t8 * width_Body) / 2.0f;
        float t50 = dalpha_Body * dgamma_Body * height_Body * m_Body * t2 * t9;
        float t52 = height_Body * t2 * t4 * t13 * 2.0f;
        float t53 = height_Body * t2 * t4 * t16 * 2.0f;
        float t58 = (height_Body * t2 * t4 * t12) / 2.0f;
        float t60 = (height_Body * t6 * t9 * t12) / 2.0f;
        float t67 = (height_Body * t2 * t4 * t13) / 2.0f;
        float t68 = (height_Body * t2 * t4 * t16) / 2.0f;
        float t70 = t2 * t8 * t9 * width_Body * (-1.0f / 2.0f);
        float t71 = t3 * t6 * t9 * width_Body * (-1.0f / 2.0f);
        float t79 = (height_Body * m_Body * t4 * t6 * t13) / 2.0f;
        float t80 = (height_Body * m_Body * t4 * t6 * t16) / 2.0f;
        float t86 = (m_Body * t4 * t9 * t16 * t17) / 4.0f;
        float t34 = r_Tau_Beta_Shoulder + t26;
        float t35 = l_Tau_Alpha_Shoulder + t27;
        float t36 = -t29;
        float t43 = t42 * 4.0f;
        float t44 = t17 * t22 * 3.0f;
        float t54 = -t42;
        float t56 = 1.0f / t38;
        float t57 = 1.0f / t41;
        float t59 = t9 * t45;
        float t61 = t9 * t46;
        float t62 = t9 * t47;
        float t63 = t9 * t48;
        float t65 = t32 + t33;
        float t69 = -t58;
        float t72 = t2 * t6 * t19 * t22;
        float t73 = t2 * t6 * t19 * t25;
        float t75 = t19 * t21 * t22 * 3.0f;
        float t76 = t19 * t21 * t25 * 3.0f;
        float t77 = t19 * t22 * t23 * 3.0f;
        float t78 = t19 * t23 * t25 * 3.0f;
        float t84 = t2 * t6 * t25 * t37;
        float t85 = t2 * t6 * t22 * t40;
        float t87 = t21 * t25 * t37 * 3.0f;
        float t88 = t21 * t22 * t40 * 3.0f;
        float t90 = t23 * t25 * t37 * 3.0f;
        float t91 = t22 * t23 * t40 * 3.0f;
        float t94 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t17 * t21) / 2.0f;
        float t95 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t17 * t23) / 2.0f;
        float t96 = (m_Body * t4 * t9 * t13 * t17 * t21) / 4.0f;
        float t97 = t21 * t86;
        float t98 = (m_Body * t4 * t9 * t13 * t17 * t23) / 4.0f;
        float t99 = t23 * t86;
        float t102 = m_Body * t4 * t9 * t16 * t17 * t21 * (-1.0f / 4.0f);
        float t104 = m_Body * t4 * t9 * t16 * t17 * t23 * (-1.0f / 4.0f);
        float t112 = t46 + t71;
        float t113 = t47 + t70;
        float t124 = t19 + t37 + t39 + t40;
        float t147 = t31 + t50 + t79 + t80;
        float t49 = t2 * t34;
        float t51 = t9 * t35;
        float t55 = -t43;
        float t64 = t4 * t6 * t34;
        float t66 = t28 + t36;
        float t81 = 1.0f / t65;
        float t83 = -t73;
        float t89 = t41 + t44;
        float t93 = -t84;
        float t100 = height_Body * t4 * t30 * t57 * 6.0f;
        float t101 = -t96;
        float t103 = -t98;
        float t105 = t45 + t63;
        float t106 = t48 + t59;
        float t107 = height_Body * t2 * t4 * t30 * t56 * 6.0f;
        float t108 = height_Body * t4 * t6 * t30 * t56 * 6.0f;
        float t109 = height_Body * t2 * t9 * t30 * t57 * 6.0f;
        float t110 = height_Body * t6 * t9 * t30 * t57 * 6.0f;
        float t111 = t3 * t9 * t30 * t57 * width_Body * 6.0f;
        float t117 = t4 * t9 * t16 * t17 * t57 * 3.0f;
        float t118 = t2 * t3 * t4 * t30 * t57 * width_Body * 6.0f;
        float t119 = t3 * t4 * t6 * t30 * t57 * width_Body * 6.0f;
        float t123 = height_Body * t3 * t4 * t9 * t30 * t57 * width_Body * 3.0f;
        float t125 = t2 * t4 * t9 * t17 * t30 * t57 * 3.0f;
        float t126 = t4 * t6 * t9 * t17 * t30 * t57 * 3.0f;
        float t127 = height_Body * t2 * t3 * t22 * t30 * t57 * width_Body * 3.0f;
        float t128 = height_Body * t2 * t3 * t25 * t30 * t57 * width_Body * 3.0f;
        float t129 = height_Body * t3 * t6 * t22 * t30 * t57 * width_Body * 3.0f;
        float t130 = height_Body * t3 * t6 * t25 * t30 * t57 * width_Body * 3.0f;
        float t135 = 1.0f / t124;
        float t138 = height_Body * t2 * t3 * t4 * t6 * t9 * t30 * t57 * width_Body * -3.0f;
        float t143 = t54 + t67 + t68;
        float t146 = t30 * t56 * t112 * 1.2e+1f;
        float t170 = t35 + t69 + t94 + t95;
        float t178 = height_Body * t2 * t4 * t30 * t56 * t147 * -6.0f;
        float t180 = t75 + t78 + t88 + t90 + t124;
        float t181 = t76 + t77 + t87 + t91 + t124;
        float t74 = -t64;
        float t82 = t6 * t66;
        float t92 = t2 * t4 * t66;
        float t114 = -t107;
        float t115 = -t108;
        float t116 = -t111;
        float t120 = -t117;
        float t121 = -t118;
        float t122 = -t119;
        float t131 = t2 * t3 * t4 * t8 * t20 * t81 * 3.0f;
        float t132 = t3 * t4 * t6 * t8 * t20 * t81 * 3.0f;
        float t133 = t20 * t22 * t24 * t81 * 3.0f;
        float t134 = t23 * t123;
        float t136 = t2 * t6 * t123;
        float t137 = t21 * t123;
        float t139 = t2 * t4 * t9 * t20 * t24 * t81 * 3.0f;
        float t140 = t4 * t6 * t9 * t20 * t24 * t81 * 3.0f;
        float t141 = t52 + t53 + t55;
        float t142 = t30 * t57 * t89;
        float t144 = t30 * t56 * t106 * 1.2e+1f;
        float t148 = t100 + t111;
        float t149 = t2 * t3 * t81 * t105 * width_Body * 6.0f;
        float t150 = t3 * t6 * t81 * t105 * width_Body * 6.0f;
        float t151 = t4 * t8 * t81 * t105 * width_Body * 6.0f;
        float t152 = t106 * t107;
        float t153 = t106 * t108;
        float t155 = t2 * t3 * t81 * t113 * width_Body * 6.0f;
        float t156 = t3 * t6 * t81 * t113 * width_Body * 6.0f;
        float t157 = t4 * t8 * t81 * t113 * width_Body * 6.0f;
        float t158 = t107 * t112;
        float t159 = t108 * t112;
        float t160 = t2 * t8 * t9 * t81 * t105 * width_Body * 6.0f;
        float t161 = t6 * t8 * t9 * t81 * t105 * width_Body * 6.0f;
        float t162 = t109 + t118;
        float t163 = t2 * t8 * t9 * t81 * t113 * width_Body * 6.0f;
        float t164 = t110 + t119;
        float t165 = t6 * t8 * t9 * t81 * t113 * width_Body * 6.0f;
        float t171 = t72 + t83 + t85 + t93;
        float t174 = t108 + t146;
        float t175 = t110 * t147;
        float t177 = t107 * t147;
        float t179 = t30 * t56 * t170 * 1.2e+1f;
        float t184 = t30 * t135 * t180;
        float t185 = t30 * t135 * t181;
        float t145 = t51 + t74 + t92;
        float t154 = t100 + t116;
        float A11 = -t123 - t133 + t142 + (t3 * t9 * t154 * width_Body) / 2.0f;
        float A12 = t125 + t127 + t157 + (t3 * t9 * t162 * width_Body) / 2.0f;
        float A13 = t126 + t129 - t151 + (t3 * t9 * t164 * width_Body) / 2.0f;
        float A14 = t123 + t133 + t142 + (t3 * t9 * t148 * width_Body) / 2.0f;
        float t166 = t109 + t121;
        float A15 = t125 - t127 - t157 + (t3 * t9 * t166 * width_Body) / 2.0f;
        float t167 = t110 + t122;
        float A16 = t126 - t129 + t151 + (t3 * t9 * t167 * width_Body) / 2.0f;
        float t168 = height_Body * t4 * t6 * t56 * t141 * (3.0f / 2.0f);
        float t169 = height_Body * t2 * t9 * t57 * t141 * (3.0f / 2.0f);
        float t172 = t107 + t144;
        float t173 = t114 + t144;
        float t176 = t115 + t146;
        float t182 = t30 * t135 * t171 * 3.0f;
        float t186 = t49 + t60 + t82 + t86 + t101 + t102 + t103 + t104;
        float t183 = -t182;
        float t187 = t30 * t57 * t186 * 1.2e+1f;
        float t188 = t168 + t178 + t179;
        float t189 = t120 + t169 + t175 + t187;
        float A17 = t126 * t147 + t100 * t186 + (t3 * t4 * t14 * width_Body) / 2.0f + (t3 * t4 * t16 * width_Body) / 2.0f + (t3 * t9 * t189 * width_Body) / 2.0f - dbeta_Body * dgamma_Body * t8 * t9 * width_Body - (height_Body * t9 * t16 * t57 * t89) / 2.0f - t4 * t8 * t81 * t145 * width_Body * 6.0f + t2 * t4 * t9 * t17 * t57 * t141 * (3.0f / 4.0f);
        float A21 = t125 - t128 + t132 - t139 - (t2 * t3 * t4 * t154 * width_Body) / 2.0f;
        float A22 = t137 - t156 + t159 + t163 + t185 + t62 * t174 - (t2 * t8 * t174 * width_Body) / 2.0f - (t2 * t3 * t4 * t162 * width_Body) / 2.0f;
        float A23 = t136 + t150 + t153 - t160 + t183 - t62 * (t107 - t144) + (t2 * t8 * width_Body * (t107 - t144)) / 2.0f - (t2 * t3 * t4 * t164 * width_Body) / 2.0f;
        float A24 = t125 + t128 - t132 + t139 - (t2 * t3 * t4 * t148 * width_Body) / 2.0f;
        float A25 = t156 - t163 + t185 - t46 * (t108 - t146) - t71 * (t108 - t146) - (t2 * t3 * t4 * t166 * width_Body) / 2.0f - height_Body * t4 * t6 * t30 * t56 * t112 * 6.0f - height_Body * t3 * t4 * t9 * t21 * t30 * t57 * width_Body * 3.0f;
        float A26 = t138 - t150 + t160 + t183 + t46 * t172 + t71 * t172 - (t2 * t3 * t4 * t167 * width_Body) / 2.0f - height_Body * t4 * t6 * t30 * t56 * t106 * 6.0f;
        float A27 = t13 * t48 + t14 * t48 + t13 * t59 + t14 * t59 + t16 * t59 + t62 * t188 + t108 * t170 + t109 * t186 + t135 * t143 * t181 - t30 * t135 * t147 * t171 * 3.0f - (t2 * t8 * t188 * width_Body) / 2.0f - dalpha_Body * dbeta_Body * t2 * t3 * width_Body - (t2 * t3 * t4 * t189 * width_Body) / 2.0f + t3 * t6 * t81 * t145 * width_Body * 6.0f - t2 * t4 * t16 * t18 * t25 * t57 * (3.0f / 2.0f) - t2 * t8 * t9 * t81 * t145 * width_Body * 6.0f - dalpha_Body * dbeta_Body * t6 * t8 * t9 * width_Body + dalpha_Body * dgamma_Body * t3 * t4 * t6 * width_Body + dbeta_Body * dgamma_Body * t2 * t4 * t8 * width_Body;
        float A31 = t126 - t130 - t131 - t140 - (t3 * t4 * t6 * t154 * width_Body) / 2.0f;
        float A32 = t136 + t155 + t165 + t183 - (t6 * t8 * t174 * width_Body) / 2.0f - (t3 * t4 * t6 * t162 * width_Body) / 2.0f - (t2 * t3 * t9 * t174 * width_Body) / 2.0f - height_Body * t2 * t4 * t30 * t56 * t112 * 6.0f;
        float A33 = t134 - t149 - t161 + t184 + (t6 * t8 * width_Body * (t107 - t144)) / 2.0f - (t3 * t4 * t6 * t164 * width_Body) / 2.0f + (t2 * t3 * t9 * width_Body * (t107 - t144)) / 2.0f - height_Body * t2 * t4 * t30 * t56 * t106 * 6.0f;
        float A34 = t126 + t130 + t131 + t140 - (t3 * t4 * t6 * t148 * width_Body) / 2.0f;
        float A35 = t138 - t155 + t158 - t165 + t183 - t48 * (t108 - t146) - t59 * (t108 - t146) - (t3 * t4 * t6 * t166 * width_Body) / 2.0f;
        float A36 = t149 + t152 + t161 + t184 + t48 * t172 + t59 * t172 - (t3 * t4 * t6 * t167 * width_Body) / 2.0f - height_Body * t3 * t4 * t9 * t23 * t30 * t57 * width_Body * 3.0f;
        float A37 = t13 * t62 + t14 * t62 + t16 * t62 + t110 * t186 + t147 * t184 - t135 * t143 * t171 * 3.0f - (t2 * t8 * t13 * width_Body) / 2.0f - (t2 * t8 * t14 * width_Body) / 2.0f - (t6 * t8 * t188 * width_Body) / 2.0f - dalpha_Body * dbeta_Body * t3 * t6 * width_Body - (t2 * t3 * t9 * t188 * width_Body) / 2.0f - (t3 * t4 * t6 * t189 * width_Body) / 2.0f - t2 * t3 * t81 * t145 * width_Body * 6.0f - height_Body * t2 * t4 * t30 * t56 * t170 * 6.0f - t4 * t6 * t16 * t18 * t25 * t57 * (3.0f / 2.0f) - t6 * t8 * t9 * t81 * t145 * width_Body * 6.0f + dalpha_Body * dbeta_Body * t2 * t8 * t9 * width_Body - dalpha_Body * dgamma_Body * t2 * t3 * t4 * width_Body + dbeta_Body * dgamma_Body * t4 * t6 * t8 * width_Body;

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
        float t15 = -r_Tau_Beta_Shoulder;
        float t9 = Mathf.Pow(t2, 2f);
        float t10 = Mathf.Pow(t3, 2f);
        float t11 = Mathf.Pow(t4, 2f);
        float t12 = Mathf.Pow(t5, 2f);
        float t16 = m_Hand * t8;
        float t19 = (g * length_Hand * m_Hand * t2) / 2.0f;
        float t17 = r_Tau_Alpha_Shoulder * t10;
        float t18 = r_Tau_Alpha_Shoulder * t12;
        float t20 = t9 * t10 * 3.0f;
        float t21 = t9 * t16 * 3.0f;
        float t22 = t9 * t12 * 3.0f;
        float t23 = t10 * t11 * t16 * 3.0f;
        float t24 = t11 * t12 * t16 * 3.0f;
        float t25 = (t2 * t4 * t7 * t16) / 4.0f;
        float t27 = (dr_Beta_Hand * dr_Alpha_Hand * t2 * t4 * t10 * t16) / 2.0f;
        float t28 = (dr_Beta_Hand * dr_Alpha_Hand * t2 * t4 * t12 * t16) / 2.0f;
        float t29 = (t2 * t4 * t6 * t10 * t16) / 4.0f;
        float t31 = (t2 * t4 * t6 * t12 * t16) / 4.0f;
        float t26 = -t25;
        float t30 = t10 * t25;
        float t32 = t12 * t25;
        float t33 = t20 + t22 + 1.0f;
        float t37 = t15 + t27 + t28;
        float t38 = t16 + t21 + t23 + t24;
        float t34 = 1.0f / t33;
        float t39 = 1.0f / t38;
        float A11 = t9 * t10 * t14 * t34 * -1.2e+1f - t8 * t11 * t12 * t39 * 1.2e+1f;
        float t45 = t17 + t18 + t19 + t26 + t29 + t30 + t31 + t32;
        float t35 = t3 * t5 * t9 * t14 * t34 * 1.2e+1f;
        float t40 = t2 * t3 * t4 * t8 * t39 * 1.2e+1f;
        float t41 = t2 * t4 * t5 * t8 * t39 * 1.2e+1f;
        float t43 = t3 * t5 * t8 * t11 * t39 * 1.2e+1f;
        float t36 = -t35;
        float t42 = -t41;
        float t44 = t36 + t43;
        float A12 = t44;
        float A13 = t42;
        float A14 = length_Hand * t2 * t5 * t6 + length_Hand * t2 * t5 * t7 + dr_Beta_Hand * dr_Alpha_Hand * length_Hand * t3 * t4 * 2.0f - length_Hand * t4 * t5 * t39 * t45 * 1.2e+1f - t2 * t3 * t13 * t14 * t34 * t37 * 1.2e+1f;
        float A21 = t44;
        float A22 = t8 * t10 * t11 * t39 * -1.2e+1f - t9 * t12 * t14 * t34 * 1.2e+1f;
        float A23 = t40;
        float A24 = -length_Hand * t2 * t3 * t6 - length_Hand * t2 * t3 * t7 + dr_Beta_Hand * dr_Alpha_Hand * length_Hand * t4 * t5 * 2.0f + length_Hand * t3 * t4 * t39 * t45 * 1.2e+1f - t2 * t5 * t13 * t14 * t34 * t37 * 1.2e+1f;
        float A31 = t42;
        float A32 = t40;
        float A33 = t8 * t9 * t39 * -1.2e+1f;
        float A34 = -length_Hand * t4 * t7 - length_Hand * t2 * t39 * t45 * 1.2e+1f;

        return new Matrix(new float[,]
        {
            { A11, A12, A13, 0, 0, 0, A14 },
            { A21, A22, A23, 0, 0, 0, A24 },
            { A31, A32, A33, 0, 0, 0, A34 }
        });
    }

    public static Matrix FFD_Coeffs_Ddr_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float depth_Body, float dgamma_Body, float g, float gamma_Body, float height_Body, float l_Alpha_Hand, float l_Tau_Beta_Shoulder, float l_Tau_Alpha_Shoulder, float m_Body, float r_Beta_Hand, float r_Tau_Beta_Shoulder, float r_Tau_Alpha_Shoulder, float width_Body)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Cos(l_Alpha_Hand);
        float t6 = Mathf.Sin(alpha_Body);
        float t7 = Mathf.Cos(r_Beta_Hand);
        float t8 = Mathf.Sin(beta_Body);
        float t9 = Mathf.Sin(gamma_Body);
        float t10 = Mathf.Sin(l_Alpha_Hand);
        float t11 = Mathf.Sin(r_Beta_Hand);
        float t12 = g * m_Body;
        float t13 = Mathf.Pow(dalpha_Body, 2f);
        float t14 = Mathf.Pow(dbeta_Body, 2f);
        float t15 = Mathf.Pow(depth_Body, 2f);
        float t16 = Mathf.Pow(dgamma_Body, 2f);
        float t17 = Mathf.Pow(height_Body, 2f);
        float t18 = Mathf.Pow(height_Body, 3f);
        float t20 = Mathf.Pow(width_Body, 2f);
        float t30 = 1.0f / m_Body;
        float t19 = Mathf.Pow(t17, 2f);
        float t21 = Mathf.Pow(t2, 2f);
        float t22 = Mathf.Pow(t4, 2f);
        float t23 = Mathf.Pow(t6, 2f);
        float t24 = Mathf.Pow(t8, 2f);
        float t25 = Mathf.Pow(t9, 2f);
        float t26 = l_Tau_Beta_Shoulder * t5;
        float t27 = r_Tau_Alpha_Shoulder * t7;
        float t28 = l_Tau_Beta_Shoulder * t10;
        float t29 = r_Tau_Alpha_Shoulder * t11;
        float t31 = -t12;
        float t32 = m_Body * t15;
        float t33 = m_Body * t20;
        float t37 = t15 * t17;
        float t38 = t15 + t17;
        float t39 = t15 * t20;
        float t40 = t17 * t20;
        float t41 = t17 + t20;
        float t42 = dalpha_Body * dgamma_Body * height_Body * t6 * t9;
        float t45 = (t2 * t3 * width_Body) / 2.0f;
        float t46 = (t2 * t8 * width_Body) / 2.0f;
        float t47 = (t3 * t6 * width_Body) / 2.0f;
        float t48 = (t6 * t8 * width_Body) / 2.0f;
        float t50 = dalpha_Body * dgamma_Body * height_Body * m_Body * t2 * t9;
        float t52 = height_Body * t2 * t4 * t13 * 2.0f;
        float t53 = height_Body * t2 * t4 * t16 * 2.0f;
        float t58 = (height_Body * t2 * t4 * t12) / 2.0f;
        float t60 = (height_Body * t6 * t9 * t12) / 2.0f;
        float t67 = (height_Body * t2 * t4 * t13) / 2.0f;
        float t68 = (height_Body * t2 * t4 * t16) / 2.0f;
        float t70 = t2 * t8 * t9 * width_Body * (-1.0f / 2.0f);
        float t71 = t3 * t6 * t9 * width_Body * (-1.0f / 2.0f);
        float t79 = (height_Body * m_Body * t4 * t6 * t13) / 2.0f;
        float t80 = (height_Body * m_Body * t4 * t6 * t16) / 2.0f;
        float t86 = (m_Body * t4 * t9 * t16 * t17) / 4.0f;
        float t34 = r_Tau_Beta_Shoulder + t26;
        float t35 = l_Tau_Alpha_Shoulder + t27;
        float t36 = -t29;
        float t43 = t42 * 4.0f;
        float t44 = t17 * t22 * 3.0f;
        float t54 = -t42;
        float t56 = 1.0f / t38;
        float t57 = 1.0f / t41;
        float t59 = t9 * t45;
        float t61 = t9 * t46;
        float t62 = t9 * t47;
        float t63 = t9 * t48;
        float t65 = t32 + t33;
        float t69 = -t58;
        float t72 = t2 * t6 * t19 * t22;
        float t73 = t2 * t6 * t19 * t25;
        float t75 = t19 * t21 * t22 * 3.0f;
        float t76 = t19 * t21 * t25 * 3.0f;
        float t77 = t19 * t22 * t23 * 3.0f;
        float t78 = t19 * t23 * t25 * 3.0f;
        float t84 = t2 * t6 * t25 * t37;
        float t85 = t2 * t6 * t22 * t40;
        float t87 = t21 * t25 * t37 * 3.0f;
        float t88 = t21 * t22 * t40 * 3.0f;
        float t90 = t23 * t25 * t37 * 3.0f;
        float t91 = t22 * t23 * t40 * 3.0f;
        float t94 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t17 * t21) / 2.0f;
        float t95 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t17 * t23) / 2.0f;
        float t96 = (m_Body * t4 * t9 * t13 * t17 * t21) / 4.0f;
        float t97 = t21 * t86;
        float t98 = (m_Body * t4 * t9 * t13 * t17 * t23) / 4.0f;
        float t99 = t23 * t86;
        float t102 = m_Body * t4 * t9 * t16 * t17 * t21 * (-1.0f / 4.0f);
        float t104 = m_Body * t4 * t9 * t16 * t17 * t23 * (-1.0f / 4.0f);
        float t112 = t46 + t71;
        float t113 = t47 + t70;
        float t124 = t19 + t37 + t39 + t40;
        float t147 = t31 + t50 + t79 + t80;
        float t49 = t2 * t34;
        float t51 = t9 * t35;
        float t55 = -t43;
        float t64 = t4 * t6 * t34;
        float t66 = t28 + t36;
        float t81 = 1.0f / t65;
        float t83 = -t73;
        float t89 = t41 + t44;
        float t93 = -t84;
        float t100 = height_Body * t4 * t30 * t57 * 6.0f;
        float t101 = -t96;
        float t103 = -t98;
        float t105 = t45 + t63;
        float t106 = t48 + t59;
        float t107 = height_Body * t2 * t4 * t30 * t56 * 6.0f;
        float t108 = height_Body * t4 * t6 * t30 * t56 * 6.0f;
        float t109 = height_Body * t2 * t9 * t30 * t57 * 6.0f;
        float t110 = height_Body * t6 * t9 * t30 * t57 * 6.0f;
        float t111 = t3 * t9 * t30 * t57 * width_Body * 6.0f;
        float t117 = t4 * t9 * t16 * t17 * t57 * 3.0f;
        float t118 = t2 * t3 * t4 * t30 * t57 * width_Body * 6.0f;
        float t119 = t3 * t4 * t6 * t30 * t57 * width_Body * 6.0f;
        float t123 = height_Body * t3 * t4 * t9 * t30 * t57 * width_Body * 3.0f;
        float t125 = t2 * t4 * t9 * t17 * t30 * t57 * 3.0f;
        float t126 = t4 * t6 * t9 * t17 * t30 * t57 * 3.0f;
        float t127 = height_Body * t2 * t3 * t22 * t30 * t57 * width_Body * 3.0f;
        float t128 = height_Body * t2 * t3 * t25 * t30 * t57 * width_Body * 3.0f;
        float t129 = height_Body * t3 * t6 * t22 * t30 * t57 * width_Body * 3.0f;
        float t130 = height_Body * t3 * t6 * t25 * t30 * t57 * width_Body * 3.0f;
        float t135 = 1.0f / t124;
        float t138 = height_Body * t2 * t3 * t4 * t6 * t9 * t30 * t57 * width_Body * -3.0f;
        float t143 = t54 + t67 + t68;
        float t146 = t30 * t56 * t112 * 1.2e+1f;
        float t170 = t35 + t69 + t94 + t95;
        float t178 = height_Body * t2 * t4 * t30 * t56 * t147 * -6.0f;
        float t180 = t75 + t78 + t88 + t90 + t124;
        float t181 = t76 + t77 + t87 + t91 + t124;
        float t74 = -t64;
        float t82 = t6 * t66;
        float t92 = t2 * t4 * t66;
        float t114 = -t107;
        float t115 = -t108;
        float t116 = -t111;
        float t120 = -t117;
        float t121 = -t118;
        float t122 = -t119;
        float t131 = t2 * t3 * t4 * t8 * t20 * t81 * 3.0f;
        float t132 = t3 * t4 * t6 * t8 * t20 * t81 * 3.0f;
        float t133 = t20 * t22 * t24 * t81 * 3.0f;
        float t134 = t23 * t123;
        float t136 = t2 * t6 * t123;
        float t137 = t21 * t123;
        float t139 = t2 * t4 * t9 * t20 * t24 * t81 * 3.0f;
        float t140 = t4 * t6 * t9 * t20 * t24 * t81 * 3.0f;
        float t141 = t52 + t53 + t55;
        float t142 = t30 * t57 * t89;
        float t144 = t30 * t56 * t106 * 1.2e+1f;
        float t148 = t100 + t111;
        float t149 = t2 * t3 * t81 * t105 * width_Body * 6.0f;
        float t150 = t3 * t6 * t81 * t105 * width_Body * 6.0f;
        float t151 = t4 * t8 * t81 * t105 * width_Body * 6.0f;
        float t152 = t106 * t107;
        float t153 = t106 * t108;
        float t155 = t2 * t3 * t81 * t113 * width_Body * 6.0f;
        float t156 = t3 * t6 * t81 * t113 * width_Body * 6.0f;
        float t157 = t4 * t8 * t81 * t113 * width_Body * 6.0f;
        float t158 = t107 * t112;
        float t159 = t108 * t112;
        float t160 = t2 * t8 * t9 * t81 * t105 * width_Body * 6.0f;
        float t161 = t6 * t8 * t9 * t81 * t105 * width_Body * 6.0f;
        float t162 = t109 + t118;
        float t163 = t2 * t8 * t9 * t81 * t113 * width_Body * 6.0f;
        float t164 = t110 + t119;
        float t165 = t6 * t8 * t9 * t81 * t113 * width_Body * 6.0f;
        float t171 = t72 + t83 + t85 + t93;
        float t174 = t108 + t146;
        float t175 = t110 * t147;
        float t177 = t107 * t147;
        float t179 = t30 * t56 * t170 * 1.2e+1f;
        float t184 = t30 * t135 * t180;
        float t185 = t30 * t135 * t181;
        float t145 = t51 + t74 + t92;
        float t154 = t100 + t116;
        float A11 = -t123 + t133 + t142 - (t3 * t9 * t154 * width_Body) / 2.0f;
        float A12 = t125 + t127 - t157 - (t3 * t9 * t162 * width_Body) / 2.0f;
        float A13 = t126 + t129 + t151 - (t3 * t9 * t164 * width_Body) / 2.0f;
        float A14 = t123 - t133 + t142 - (t3 * t9 * t148 * width_Body) / 2.0f;
        float t166 = t109 + t121;
        float A15 = t125 - t127 + t157 - (t3 * t9 * t166 * width_Body) / 2.0f;
        float t167 = t110 + t122;
        float A16 = t126 - t129 - t151 - (t3 * t9 * t167 * width_Body) / 2.0f;
        float t168 = height_Body * t4 * t6 * t56 * t141 * (3.0f / 2.0f);
        float t169 = height_Body * t2 * t9 * t57 * t141 * (3.0f / 2.0f);
        float t172 = t107 + t144;
        float t173 = t114 + t144;
        float t176 = t115 + t146;
        float t182 = t30 * t135 * t171 * 3.0f;
        float t186 = t49 + t60 + t82 + t86 + t101 + t102 + t103 + t104;
        float t183 = -t182;
        float t187 = t30 * t57 * t186 * 1.2e+1f;
        float t188 = t168 + t178 + t179;
        float t189 = t120 + t169 + t175 + t187;
        float A17 = t126 * t147 + t100 * t186 - (t3 * t4 * t14 * width_Body) / 2.0f - (t3 * t4 * t16 * width_Body) / 2.0f - (t3 * t9 * t189 * width_Body) / 2.0f + dbeta_Body * dgamma_Body * t8 * t9 * width_Body - (height_Body * t9 * t16 * t57 * t89) / 2.0f + t4 * t8 * t81 * t145 * width_Body * 6.0f + t2 * t4 * t9 * t17 * t57 * t141 * (3.0f / 4.0f);
        float A21 = t125 - t128 - t132 + t139 + t4 * t45 * t154;
        float A22 = t137 + t156 + t159 - t163 + t185 + t46 * t174 + t71 * t174 + t4 * t45 * t162;
        float A23 = t136 - t150 + t153 + t160 + t183 - t46 * (t107 - t144) - t71 * (t107 - t144) + t4 * t45 * t164;
        float A24 = t125 + t128 + t132 - t139 + t4 * t45 * t148;
        float A25 = -t156 + t163 + t185 - t62 * (t108 - t146) + t4 * t45 * t166 + (t2 * t8 * width_Body * (t108 - t146)) / 2.0f - height_Body * t4 * t6 * t30 * t56 * t112 * 6.0f - height_Body * t3 * t4 * t9 * t21 * t30 * t57 * width_Body * 3.0f;
        float A26 = t138 + t150 - t160 + t183 + t62 * t172 + t4 * t45 * t167 - (t2 * t8 * t172 * width_Body) / 2.0f - height_Body * t4 * t6 * t30 * t56 * t106 * 6.0f;
        float A27 = t46 * t188 + t71 * t188 + t108 * t170 + t109 * t186 + t4 * t45 * t189 + t135 * t143 * t181 - t30 * t135 * t147 * t171 * 3.0f - (t6 * t8 * t13 * width_Body) / 2.0f - (t6 * t8 * t14 * width_Body) / 2.0f + dalpha_Body * dbeta_Body * t2 * t3 * width_Body - (t2 * t3 * t9 * t13 * width_Body) / 2.0f - (t2 * t3 * t9 * t14 * width_Body) / 2.0f - (t2 * t3 * t9 * t16 * width_Body) / 2.0f - t3 * t6 * t81 * t145 * width_Body * 6.0f - t2 * t4 * t16 * t18 * t25 * t57 * (3.0f / 2.0f) + t2 * t8 * t9 * t81 * t145 * width_Body * 6.0f + dalpha_Body * dbeta_Body * t6 * t8 * t9 * width_Body - dalpha_Body * dgamma_Body * t3 * t4 * t6 * width_Body - dbeta_Body * dgamma_Body * t2 * t4 * t8 * width_Body;
        float A31 = t126 - t130 + t131 + t140 + t4 * t47 * t154;
        float A32 = t136 - t155 - t165 + t183 + t48 * t174 + t59 * t174 + t4 * t47 * t162 - height_Body * t2 * t4 * t30 * t56 * t112 * 6.0f;
        float A33 = t134 + t149 + t161 + t184 - t48 * (t107 - t144) - t59 * (t107 - t144) + t4 * t47 * t164 - height_Body * t2 * t4 * t30 * t56 * t106 * 6.0f;
        float A34 = t126 + t130 - t131 - t140 + t4 * t47 * t148;
        float A35 = t138 + t155 + t158 + t165 + t183 + t4 * t47 * t166 + (t6 * t8 * width_Body * (t108 - t146)) / 2.0f + (t2 * t3 * t9 * width_Body * (t108 - t146)) / 2.0f;
        float A36 = -t149 + t152 - t161 + t184 + t4 * t47 * t167 - (t6 * t8 * t172 * width_Body) / 2.0f - (t2 * t3 * t9 * t172 * width_Body) / 2.0f - height_Body * t3 * t4 * t9 * t23 * t30 * t57 * width_Body * 3.0f;
        float A37 = t13 * t46 + t14 * t46 + t13 * t71 + t14 * t71 + t16 * t71 + t48 * t188 + t59 * t188 + t110 * t186 + t147 * t184 + t4 * t47 * t189 - t135 * t143 * t171 * 3.0f + dalpha_Body * dbeta_Body * t3 * t6 * width_Body + t2 * t3 * t81 * t145 * width_Body * 6.0f - height_Body * t2 * t4 * t30 * t56 * t170 * 6.0f - t4 * t6 * t16 * t18 * t25 * t57 * (3.0f / 2.0f) + t6 * t8 * t9 * t81 * t145 * width_Body * 6.0f - dalpha_Body * dbeta_Body * t2 * t8 * t9 * width_Body + dalpha_Body * dgamma_Body * t2 * t3 * t4 * width_Body - dbeta_Body * dgamma_Body * t4 * t6 * t8 * width_Body;

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
        float t12 = Mathf.Pow(t5, 2f);
        float ddl_Alpha_Hand = ((l_Tau_Alpha_Shoulder + l_F_Z * length_Hand * t2 + (g * length_Hand * m_Hand * t2) / 2.0f - l_F_Y * length_Hand * t3 * t4 - l_F_X * length_Hand * t4 * t5 - (m_Hand * t2 * t4 * t7 * t8) / 4.0f + (m_Hand * t2 * t4 * t6 * t8 * t10) / 4.0f + (m_Hand * t2 * t4 * t7 * t8 * t10) / 4.0f + (m_Hand * t2 * t4 * t6 * t8 * t12) / 4.0f + (m_Hand * t2 * t4 * t7 * t8 * t12) / 4.0f) * -1.2e+1f) / (m_Hand * t8 + m_Hand * t8 * t9 * 3.0f + m_Hand * t8 * t10 * t11 * 3.0f + m_Hand * t8 * t11 * t12 * 3.0f);
        float ddl_Beta_Hand = ((l_Tau_Beta_Shoulder * t9 + l_Tau_Beta_Shoulder * t11 + l_F_X * length_Hand * t2 * t3 - l_F_Y * length_Hand * t2 * t5 - (dl_Beta_Hand * dl_Alpha_Hand * m_Hand * t2 * t4 * t8 * t10) / 2.0f - (dl_Beta_Hand * dl_Alpha_Hand * m_Hand * t2 * t4 * t8 * t12) / 2.0f) * -1.2e+1f) / (m_Hand * t8 * (t9 * t10 * 3.0f + t9 * t12 * 3.0f + 1.0f));

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
        float t12 = Mathf.Pow(t5, 2f);
        float ddr_Alpha_Hand = ((r_Tau_Alpha_Shoulder * t10 + r_Tau_Alpha_Shoulder * t12 + length_Hand * r_F_Z * t2 + (g * length_Hand * m_Hand * t2) / 2.0f - length_Hand * r_F_Y * t3 * t4 + length_Hand * r_F_X * t4 * t5 - (m_Hand * t2 * t4 * t7 * t8) / 4.0f + (m_Hand * t2 * t4 * t6 * t8 * t10) / 4.0f + (m_Hand * t2 * t4 * t7 * t8 * t10) / 4.0f + (m_Hand * t2 * t4 * t6 * t8 * t12) / 4.0f + (m_Hand * t2 * t4 * t7 * t8 * t12) / 4.0f) * -1.2e+1f) / (m_Hand * t8 + m_Hand * t8 * t9 * 3.0f + m_Hand * t8 * t10 * t11 * 3.0f + m_Hand * t8 * t11 * t12 * 3.0f);
        float ddr_Beta_Hand = ((-r_Tau_Beta_Shoulder + length_Hand * r_F_X * t2 * t3 + length_Hand * r_F_Y * t2 * t5 + (dr_Beta_Hand * dr_Alpha_Hand * m_Hand * t2 * t4 * t8 * t10) / 2.0f + (dr_Beta_Hand * dr_Alpha_Hand * m_Hand * t2 * t4 * t8 * t12) / 2.0f) * 1.2e+1f) / (m_Hand * t8 * (t9 * t10 * 3.0f + t9 * t12 * 3.0f + 1.0f));

        return new float[] { ddr_Alpha_Hand, ddr_Beta_Hand };

    }

    public static float[] FFD_Dds_Body(float alpha_Body, float beta_Body, float dalpha_Body, float depth_Body, float dgamma_Body, float g, float gamma_Body, float height_Body, float l_Alpha_Hand, float l_F_X, float l_F_Y, float l_F_Z, float l_Tau_Beta_Shoulder, float l_Tau_Alpha_Shoulder, float m_Body, float r_Beta_Hand, float r_F_X, float r_F_Y, float r_F_Z, float r_Tau_Beta_Shoulder, float r_Tau_Alpha_Shoulder, float width_Body)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Cos(l_Alpha_Hand);
        float t6 = Mathf.Sin(alpha_Body);
        float t7 = Mathf.Cos(r_Beta_Hand);
        float t8 = Mathf.Sin(beta_Body);
        float t9 = Mathf.Sin(gamma_Body);
        float t10 = Mathf.Sin(l_Alpha_Hand);
        float t11 = Mathf.Sin(r_Beta_Hand);
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
        float t24 = r_Tau_Alpha_Shoulder * t7;
        float t25 = l_Tau_Beta_Shoulder * t10;
        float t26 = r_Tau_Alpha_Shoulder * t11;
        float t28 = -t12;
        float t31 = t14 * t16;
        float t32 = t14 + t16;
        float t33 = t14 * t18;
        float t34 = t16 * t18;
        float t35 = t16 + t18;
        float t36 = dalpha_Body * dgamma_Body * height_Body * t6 * t9 * 4.0f;
        float t37 = (t2 * t3 * width_Body) / 2.0f;
        float t38 = (t2 * t8 * width_Body) / 2.0f;
        float t39 = (t3 * t6 * width_Body) / 2.0f;
        float t40 = (t6 * t8 * width_Body) / 2.0f;
        float t42 = dalpha_Body * dgamma_Body * height_Body * m_Body * t2 * t9;
        float t43 = height_Body * t2 * t4 * t13 * 2.0f;
        float t44 = height_Body * t2 * t4 * t15 * 2.0f;
        float t48 = (l_F_X * t3 * t9 * width_Body) / 2.0f;
        float t49 = (r_F_X * t3 * t9 * width_Body) / 2.0f;
        float t50 = (height_Body * t2 * t4 * t12) / 2.0f;
        float t52 = (height_Body * t6 * t9 * t12) / 2.0f;
        float t57 = (height_Body * m_Body * t9 * t15) / 2.0f;
        float t60 = t2 * t8 * t9 * width_Body * (-1.0f / 2.0f);
        float t61 = t3 * t6 * t9 * width_Body * (-1.0f / 2.0f);
        float t69 = (height_Body * m_Body * t4 * t6 * t13) / 2.0f;
        float t70 = (height_Body * m_Body * t4 * t6 * t15) / 2.0f;
        float t71 = l_F_Y * t2 * t3 * t4 * width_Body * (-1.0f / 2.0f);
        float t72 = l_F_Z * t3 * t4 * t6 * width_Body * (-1.0f / 2.0f);
        float t78 = (m_Body * t4 * t9 * t15 * t16) / 4.0f;
        float t29 = r_Tau_Beta_Shoulder + t23;
        float t30 = -t26;
        float t45 = -t36;
        float t46 = 1.0f / t32;
        float t47 = 1.0f / t35;
        float t51 = t9 * t37;
        float t53 = t9 * t38;
        float t54 = t9 * t39;
        float t55 = t9 * t40;
        float t58 = -t49;
        float t59 = -t50;
        float t62 = l_F_Y * t4 * t37;
        float t63 = r_F_Y * t4 * t37;
        float t64 = l_F_Z * t4 * t39;
        float t65 = r_F_Z * t4 * t39;
        float t66 = -t57;
        float t67 = t2 * t6 * t17 * t20;
        float t68 = t2 * t6 * t17 * t22;
        float t76 = t2 * t6 * t22 * t31;
        float t77 = t2 * t6 * t20 * t34;
        float t80 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t16 * t19) / 2.0f;
        float t81 = (dalpha_Body * dgamma_Body * m_Body * t4 * t9 * t16 * t21) / 2.0f;
        float t82 = (m_Body * t4 * t9 * t13 * t16 * t19) / 4.0f;
        float t83 = t19 * t78;
        float t84 = (m_Body * t4 * t9 * t13 * t16 * t21) / 4.0f;
        float t85 = t21 * t78;
        float t87 = m_Body * t4 * t9 * t15 * t16 * t19 * (-1.0f / 4.0f);
        float t89 = m_Body * t4 * t9 * t15 * t16 * t21 * (-1.0f / 4.0f);
        float t92 = t38 + t61;
        float t93 = t39 + t60;
        float t98 = t17 + t31 + t33 + t34;
        float t105 = l_F_Z + r_F_Z + t28 + t42 + t69 + t70;
        float t41 = t2 * t29;
        float t56 = t25 + t30;
        float t74 = -t68;
        float t75 = l_F_X + r_F_X + t66;
        float t79 = -t76;
        float t86 = -t82;
        float t88 = -t84;
        float t90 = t37 + t55;
        float t91 = t40 + t51;
        float t96 = l_F_Y * t92;
        float t97 = r_F_Y * t92;
        float t101 = 1.0f / t98;
        float t102 = t43 + t44 + t45;
        float t73 = t6 * t56;
        float t94 = l_F_Z * t91;
        float t95 = r_F_Z * t91;
        float t100 = -t96;
        float t103 = (m_Body * t102) / 4.0f;
        float t106 = t67 + t74 + t77 + t79;
        float t99 = -t94;
        float t104 = l_F_Y + r_F_Y + t103;
        float t108 = t41 + t48 + t52 + t58 + t63 + t65 + t71 + t72 + t73 + t78 + t86 + t87 + t88 + t89;
        float t107 = l_Tau_Alpha_Shoulder + t24 + t59 + t80 + t81 + t95 + t97 + t99 + t100;
        float ddalpha_Body = t27 * t46 * t107 * 1.2e+1f - height_Body * t2 * t4 * t27 * t46 * t105 * 6.0f + height_Body * t4 * t6 * t27 * t46 * t104 * 6.0f;
        float ddbeta_Body = ((t9 * (l_Tau_Alpha_Shoulder + t24) - l_F_Z * t90 + l_F_Y * t93 + r_F_Z * t90 - r_F_Y * t93 - t4 * t6 * t29 + t2 * t4 * t56 - (l_F_X * t4 * t8 * width_Body) / 2.0f + (r_F_X * t4 * t8 * width_Body) / 2.0f) * -1.2e+1f) / (m_Body * t14 + m_Body * t18);
        float ddgamma_Body = t27 * t47 * t108 * 1.2e+1f + height_Body * t4 * t27 * t47 * t75 * 6.0f + height_Body * t2 * t9 * t27 * t47 * t104 * 6.0f + height_Body * t6 * t9 * t27 * t47 * t105 * 6.0f;
        float ddx_Head = t27 * t47 * t75 * (t35 + t16 * t20 * 3.0f) + height_Body * t4 * t27 * t47 * t108 * 6.0f + t2 * t4 * t9 * t16 * t27 * t47 * t104 * 3.0f + t4 * t6 * t9 * t16 * t27 * t47 * t105 * 3.0f;
        float ddy_Head = t27 * t101 * t105 * t106 * -3.0f + t27 * t101 * t104 * (t98 + t17 * t19 * t22 * 3.0f + t17 * t20 * t21 * 3.0f + t19 * t22 * t31 * 3.0f + t20 * t21 * t34 * 3.0f) + height_Body * t4 * t6 * t27 * t46 * t107 * 6.0f + height_Body * t2 * t9 * t27 * t47 * t108 * 6.0f + t2 * t4 * t9 * t16 * t27 * t47 * t75 * 3.0f;
        float ddz_Head = t27 * t101 * t104 * t106 * -3.0f + t27 * t101 * t105 * (t98 + t17 * t19 * t20 * 3.0f + t17 * t21 * t22 * 3.0f + t19 * t20 * t34 * 3.0f + t21 * t22 * t31 * 3.0f) - height_Body * t2 * t4 * t27 * t46 * t107 * 6.0f + height_Body * t6 * t9 * t27 * t47 * t108 * 6.0f + t4 * t6 * t9 * t16 * t27 * t47 * t75 * 3.0f;

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
            { ddl_Beta_Hand * length_Hand * t2 * t3 - ddl_Alpha_Hand * length_Hand * t4 * t5 - length_Hand * t2 * t5 * t6 - length_Hand * t2 * t5 * t7 - dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t3 * t4 * 2.0f, -ddl_Beta_Hand * length_Hand * t2 * t5 - ddl_Alpha_Hand * length_Hand * t3 * t4 - length_Hand * t2 * t3 * t6 - length_Hand * t2 * t3 * t7 + dl_Beta_Hand * dl_Alpha_Hand * length_Hand * t4 * t5 * 2.0f, ddl_Alpha_Hand * length_Hand * t2 - length_Hand * t4 * t7 }
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
            { -ddr_Beta_Hand * length_Hand * t2 * t3 + ddr_Alpha_Hand * length_Hand * t4 * t5 + length_Hand * t2 * t5 * t6 + length_Hand * t2 * t5 * t7 + dr_Beta_Hand * dr_Alpha_Hand * length_Hand * t3 * t4 * 2.0f, -ddr_Beta_Hand * length_Hand * t2 * t5 - ddr_Alpha_Hand * length_Hand * t3 * t4 - length_Hand * t2 * t3 * t6 - length_Hand * t2 * t3 * t7 + dr_Beta_Hand * dr_Alpha_Hand * length_Hand * t4 * t5 * 2.0f, ddr_Alpha_Hand * length_Hand * t2 - length_Hand * t4 * t7 }
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
            { dl_Beta_Hand* length_Hand * t2* t3 - dl_Alpha_Hand* length_Hand * t4* t5, -dl_Beta_Hand* length_Hand * t2* t5 - dl_Alpha_Hand* length_Hand * t3* t4, dl_Alpha_Hand * length_Hand* t2}
        }));
    }

    public static Matrix FRD_Dl_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float dgamma_Body, float dx_Head, float dy_Head, float dz_Head, float gamma_Body, float width_Body)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { dx_Head + (dbeta_Body * t4 * t6 * width_Body) / 2.0f + (dgamma_Body * t3 * t7 * width_Body) / 2.0f, dy_Head - (dalpha_Body * t2 * t6 * width_Body) / 2.0f - (dbeta_Body * t3 * t5 * width_Body) / 2.0f + (dalpha_Body * t3 * t5 * t7 * width_Body) / 2.0f + (dbeta_Body * t2 * t6 * t7 * width_Body) / 2.0f - (dgamma_Body * t2 * t3 * t4 * width_Body) / 2.0f, dz_Head - (dalpha_Body * t5 * t6 * width_Body) / 2.0f + (dbeta_Body * t2 * t3 * width_Body) / 2.0f - (dalpha_Body * t2 * t3 * t7 * width_Body) / 2.0f + (dbeta_Body * t5 * t6 * t7 * width_Body) / 2.0f - (dgamma_Body * t3 * t4 * t5 * width_Body) / 2.0f }
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
            { -dr_Beta_Hand * length_Hand * t2 * t3 + dr_Alpha_Hand * length_Hand * t4 * t5, -dr_Beta_Hand * length_Hand * t2 * t5 - dr_Alpha_Hand * length_Hand * t3 * t4, dr_Alpha_Hand * length_Hand * t2 }
        }));

    }

    public static Matrix FRD_Dr_Shoulder(float alpha_Body, float beta_Body, float dalpha_Body, float dbeta_Body, float dgamma_Body, float dx_Head, float dy_Head, float dz_Head, float gamma_Body, float width_Body)
    {

        float t2 = Mathf.Cos(alpha_Body);
        float t3 = Mathf.Cos(beta_Body);
        float t4 = Mathf.Cos(gamma_Body);
        float t5 = Mathf.Sin(alpha_Body);
        float t6 = Mathf.Sin(beta_Body);
        float t7 = Mathf.Sin(gamma_Body);
        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { dx_Head - (dbeta_Body * t4 * t6 * width_Body) / 2.0f - (dgamma_Body * t3 * t7 * width_Body) / 2.0f, dy_Head + (dalpha_Body * t2 * t6 * width_Body) / 2.0f + (dbeta_Body * t3 * t5 * width_Body) / 2.0f - (dalpha_Body * t3 * t5 * t7 * width_Body) / 2.0f - (dbeta_Body * t2 * t6 * t7 * width_Body) / 2.0f + (dgamma_Body * t2 * t3 * t4 * width_Body) / 2.0f, dz_Head + (dalpha_Body * t5 * t6 * width_Body) / 2.0f - (dbeta_Body * t2 * t3 * width_Body) / 2.0f + (dalpha_Body * t2 * t3 * t7 * width_Body) / 2.0f - (dbeta_Body * t5 * t6 * t7 * width_Body) / 2.0f + (dgamma_Body * t3 * t4 * t5 * width_Body) / 2.0f }
        }));

    }

    public static Matrix FRD_L_Arm_Bottom(float l_Alpha_Hand, float l_Beta_Hand, float l_X_Fixed, float l_Y_Fixed, float l_Z_Fixed, float length_Hand)
    {
        float t2 = Mathf.Cos(l_Alpha_Hand);
        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { l_X_Fixed + length_Hand * t2 * Mathf.Sin(l_Beta_Hand), l_Y_Fixed + length_Hand * t2 * Mathf.Cos(l_Beta_Hand), l_Z_Fixed + length_Hand * Mathf.Sin(l_Alpha_Hand) }
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
        }));
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
        float t2 = Mathf.Cos(r_Alpha_Hand);
        return Matrix.MatrixTranspose(new Matrix(new float[,]
        {
            { r_X_Fixed - length_Hand * t2 * Mathf.Sin(r_Beta_Hand), r_Y_Fixed + length_Hand * t2 * Mathf.Cos(r_Beta_Hand), r_Z_Fixed + length_Hand * Mathf.Sin(r_Alpha_Hand) }
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
