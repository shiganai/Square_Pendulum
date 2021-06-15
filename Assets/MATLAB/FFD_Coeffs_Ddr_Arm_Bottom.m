function [A11,A12,A13,A14,A21,A22,A23,A24,A31,A32,A33,A34] = FFD_Coeffs_Ddr_Arm_Bottom(dr_Beta_Hand,dr_Alpha_Hand,g,length_Hand,m_Hand,r_Alpha_Hand,r_Beta_Hand,r_Tau_Beta_Shoulder,r_Tau_Alpha_Shoulder)
%FFD_COEFFS_DDR_ARM_BOTTOM
%    [A11,A12,A13,A14,A21,A22,A23,A24,A31,A32,A33,A34] = FFD_COEFFS_DDR_ARM_BOTTOM(DR_BETA_HAND,DR_ALPHA_HAND,G,LENGTH_HAND,M_HAND,R_ALPHA_HAND,R_BETA_HAND,R_TAU_BETA_SHOULDER,R_TAU_ALPHA_SHOULDER)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    12-Jun-2021 22:17:14

t2 = cos(r_Alpha_Hand);
t3 = cos(r_Beta_Hand);
t4 = sin(r_Alpha_Hand);
t5 = sin(r_Beta_Hand);
t6 = dr_Beta_Hand.^2;
t7 = dr_Alpha_Hand.^2;
t8 = length_Hand.^2;
t32 = -r_Tau_Alpha_Shoulder;
t9 = t2.^2;
t10 = t2.^3;
t12 = t2.^5;
t14 = t2.^7;
t15 = t3.^2;
t16 = t3.^3;
t18 = t3.^5;
t20 = t3.^7;
t21 = t4.^2;
t22 = t4.^3;
t24 = t4.^5;
t26 = t4.^7;
t27 = t5.^2;
t28 = t5.^3;
t30 = t5.^5;
t36 = t3.^11;
t52 = (g.*length_Hand.*m_Hand.*t2.*t3)./2.0;
t54 = (g.*length_Hand.*m_Hand.*t4.*t5)./2.0;
t85 = (m_Hand.*t3.*t5.*t6.*t8)./4.0;
t11 = t9.^2;
t13 = t9.^3;
t17 = t15.^2;
t19 = t15.^3;
t23 = t21.^2;
t25 = t21.^3;
t29 = t27.^2;
t31 = t27.^3;
t35 = t16.^3;
t39 = t27.^5;
t40 = r_Tau_Beta_Shoulder.*t9;
t41 = r_Tau_Beta_Shoulder.*t21;
t42 = t15.*3.0;
t44 = t2.*t3.*t22;
t49 = t9.*t27.*3.0;
t51 = t21.*t27.*3.0;
t55 = t2.*t16.*t22;
t56 = t4.*t10.*t16;
t57 = t4.*t10.*t18;
t60 = -t52;
t61 = -t54;
t63 = t9.*t21.*t27;
t66 = t3.*t4.*t10.*t27;
t72 = t9.*t21.*t28.*2.0;
t77 = t3.*t4.*t10.*t28.*2.0;
t79 = t3.*t4.*t10.*t30.*2.0;
t93 = -t85;
t95 = m_Hand.*t2.*t8.*t16.*t26.*4.0;
t96 = m_Hand.*t2.*t8.*t18.*t26.*4.0;
t97 = t5.*t9.*t15.*t21.*2.0;
t104 = m_Hand.*t2.*t8.*t16.*t24.*1.2e+1;
t106 = m_Hand.*t2.*t8.*t18.*t24.*1.2e+1;
t107 = m_Hand.*t4.*t8.*t12.*t18.*1.2e+1;
t108 = m_Hand.*t4.*t8.*t14.*t36.*4.0;
t109 = m_Hand.*t4.*t8.*t12.*t20.*1.2e+1;
t111 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t5.*t8.*t9)./2.0;
t116 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t5.*t8.*t21)./2.0;
t119 = m_Hand.*t2.*t3.*t8.*t26.*t27.*4.0;
t124 = m_Hand.*t8.*t10.*t16.*t22.*1.2e+1;
t125 = m_Hand.*t8.*t10.*t18.*t24.*1.2e+1;
t126 = m_Hand.*t8.*t10.*t20.*t22.*1.2e+1;
t127 = m_Hand.*t8.*t10.*t20.*t24.*1.2e+1;
t128 = m_Hand.*t8.*t12.*t20.*t22.*1.2e+1;
t139 = t9.*t85;
t140 = (m_Hand.*t3.*t5.*t7.*t8.*t9)./4.0;
t147 = (m_Hand.*t2.*t5.*t7.*t8.*t22)./6.0;
t148 = t21.*t85;
t149 = (m_Hand.*t3.*t5.*t7.*t8.*t21)./4.0;
t165 = (m_Hand.*t2.*t5.*t6.*t8.*t22)./2.4e+1;
t167 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t8.*t9.*t21)./4.0;
t172 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t2.*t8.*t22.*t27)./6.0;
t173 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t4.*t8.*t10.*t27)./6.0;
t182 = m_Hand.*t2.*t8.*t16.*t26.*t27.*8.0;
t184 = m_Hand.*t4.*t8.*t12.*t16.*t27.*1.2e+1;
t190 = m_Hand.*t4.*t8.*t14.*t20.*t27.*1.6e+1;
t191 = m_Hand.*t4.*t8.*t12.*t18.*t27.*2.4e+1;
t197 = (m_Hand.*t2.*t7.*t8.*t15.*t22)./6.0;
t206 = (m_Hand.*t2.*t6.*t8.*t22.*t27)./6.0;
t210 = (m_Hand.*t2.*t7.*t8.*t22.*t28)./6.0;
t211 = (m_Hand.*t4.*t7.*t8.*t10.*t28)./6.0;
t212 = (m_Hand.*t4.*t7.*t8.*t10.*t30)./6.0;
t219 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t8.*t9.*t16.*t21)./2.0;
t220 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t8.*t9.*t18.*t21)./4.0;
t235 = (m_Hand.*t3.*t6.*t8.*t9.*t21)./8.0;
t236 = (m_Hand.*t2.*t6.*t8.*t15.*t22)./1.2e+1;
t249 = (m_Hand.*t4.*t6.*t8.*t10.*t27)./1.2e+1;
t251 = (m_Hand.*t2.*t6.*t8.*t22.*t28)./2.4e+1;
t252 = (m_Hand.*t4.*t6.*t8.*t10.*t28)./2.4e+1;
t253 = (m_Hand.*t4.*t6.*t8.*t10.*t30)./2.4e+1;
t259 = (m_Hand.*t6.*t8.*t9.*t16.*t21)./3.0;
t275 = m_Hand.*t8.*t10.*t16.*t24.*t27.*2.4e+1;
t276 = m_Hand.*t8.*t10.*t18.*t22.*t27.*2.4e+1;
t281 = m_Hand.*t8.*t10.*t18.*t24.*t27.*3.6e+1;
t283 = m_Hand.*t8.*t12.*t18.*t22.*t27.*3.6e+1;
t286 = m_Hand.*t8.*t12.*t20.*t22.*t27.*4.8e+1;
t289 = (m_Hand.*t4.*t5.*t7.*t8.*t10.*t15)./6.0;
t303 = (m_Hand.*t6.*t8.*t9.*t18.*t21)./8.0;
t312 = (m_Hand.*t3.*t5.*t6.*t8.*t9.*t21)./1.2e+1;
t314 = (m_Hand.*t4.*t5.*t6.*t8.*t10.*t15)./2.4e+1;
t316 = m_Hand.*t3.*t5.*t7.*t8.*t9.*t21.*(-1.0./4.0);
t319 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t2.*t8.*t15.*t22.*t27)./3.0;
t320 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t8.*t9.*t21.*t28)./3.0;
t321 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t5.*t8.*t9.*t16.*t21)./3.0;
t322 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t4.*t8.*t10.*t15.*t27)./3.0;
t329 = m_Hand.*t2.*t5.*t6.*t8.*t15.*t22.*(-1.0./2.4e+1);
t333 = (m_Hand.*t3.*t6.*t8.*t9.*t21.*t28)./6.0;
t334 = (m_Hand.*t5.*t6.*t8.*t9.*t16.*t21)./6.0;
t335 = (m_Hand.*t4.*t6.*t8.*t10.*t15.*t27)./6.0;
t338 = m_Hand.*t3.*t7.*t8.*t9.*t21.*t28.*(2.0./3.0);
t339 = m_Hand.*t5.*t7.*t8.*t9.*t16.*t21.*(2.0./3.0);
t340 = (m_Hand.*t4.*t7.*t8.*t10.*t15.*t28)./3.0;
t343 = (m_Hand.*t3.*t7.*t8.*t9.*t21.*t30)./4.0;
t344 = (m_Hand.*t5.*t7.*t8.*t9.*t18.*t21)./4.0;
t345 = (m_Hand.*t4.*t7.*t8.*t10.*t15.*t27)./6.0;
t357 = (m_Hand.*t4.*t6.*t8.*t10.*t15.*t28)./1.2e+1;
t359 = (m_Hand.*t3.*t6.*t8.*t9.*t21.*t30)./1.2e+1;
t360 = (m_Hand.*t5.*t6.*t8.*t9.*t18.*t21)./1.2e+1;
t365 = (m_Hand.*t6.*t8.*t9.*t16.*t21.*t28)./6.0;
t366 = (m_Hand.*t7.*t8.*t9.*t16.*t21.*t28)./2.0;
t33 = t11.^2;
t34 = t17.^2;
t37 = t23.^2;
t38 = t29.^2;
t43 = t5.*t23;
t45 = t11.*t19;
t46 = t15.*t23;
t47 = t11.*t30;
t48 = t9.*t42;
t50 = t21.*t42;
t53 = -t44;
t58 = t5.*t11.*t17;
t59 = t5.*t44.*2.0;
t62 = t11.*t15.*t29;
t64 = t9.*t21.*t31;
t65 = t27.*t44;
t67 = t3.*t4.*t10.*t29;
t68 = -t56;
t69 = t9.*t17.*t21.*2.0;
t70 = t11.*t15.*t28.*2.0;
t71 = t11.*t17.*t27.*2.0;
t73 = t9.*t21.*t29.*2.0;
t75 = t28.*t44.*2.0;
t76 = t5.*t55.*2.0;
t78 = t5.*t56.*2.0;
t80 = t5.*t57.*2.0;
t81 = m_Hand.*t8.*t9.*t17.*3.6e+1;
t83 = m_Hand.*t8.*t17.*t23.*1.2e+1;
t84 = m_Hand.*t8.*t17.*t21.*3.6e+1;
t87 = -t66;
t90 = -t79;
t98 = t27.*t56.*2.0;
t99 = t28.*t56.*4.0;
t101 = t17.*t63;
t102 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t8.*t23)./1.2e+1;
t103 = -t95;
t105 = m_Hand.*t4.*t8.*t14.*t35.*4.0;
t112 = t15.*t63.*2.0;
t114 = m_Hand.*t8.*t9.*t25.*t29.*4.0;
t115 = m_Hand.*t8.*t11.*t23.*t31.*6.0;
t118 = (m_Hand.*t3.*t6.*t8.*t23)./1.2e+1;
t120 = m_Hand.*t2.*t3.*t8.*t26.*t29.*4.0;
t121 = -t104;
t123 = -t107;
t129 = m_Hand.*t8.*t9.*t19.*t21.*2.4e+1;
t132 = m_Hand.*t8.*t11.*t17.*t29.*1.2e+1;
t134 = m_Hand.*t8.*t13.*t15.*t31.*1.2e+1;
t135 = m_Hand.*t8.*t13.*t19.*t27.*1.2e+1;
t137 = m_Hand.*t8.*t13.*t17.*t29.*2.4e+1;
t138 = m_Hand.*t8.*t11.*t15.*t27.*3.6e+1;
t145 = m_Hand.*t8.*t15.*t25.*t27.*1.2e+1;
t151 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t8.*t11.*t16)./1.2e+1;
t152 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t8.*t11.*t18)./1.2e+1;
t153 = -t119;
t155 = m_Hand.*t3.*t4.*t8.*t14.*t39.*4.0;
t156 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t8.*t16.*t23)./1.2e+1;
t157 = -t124;
t158 = -t125;
t159 = -t128;
t160 = m_Hand.*t8.*t12.*t22.*t35.*1.2e+1;
t166 = -t147;
t169 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t5.*t8.*t11.*t18)./6.0;
t170 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t2.*t8.*t22.*t29)./3.0;
t171 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t4.*t8.*t10.*t29)./3.0;
t174 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t2.*t8.*t22.*t31)./6.0;
t175 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t4.*t8.*t10.*t31)./6.0;
t176 = (m_Hand.*t6.*t8.*t11.*t18)./1.2e+1;
t177 = (m_Hand.*t6.*t8.*t11.*t16)./2.4e+1;
t180 = (m_Hand.*t6.*t8.*t16.*t23)./2.4e+1;
t183 = t27.*t104;
t185 = m_Hand.*t3.*t8.*t10.*t24.*t29.*1.2e+1;
t186 = m_Hand.*t4.*t8.*t12.*t16.*t29.*1.2e+1;
t187 = m_Hand.*t3.*t8.*t10.*t24.*t31.*1.2e+1;
t188 = m_Hand.*t3.*t8.*t12.*t22.*t31.*1.2e+1;
t189 = m_Hand.*t4.*t8.*t14.*t16.*t31.*1.6e+1;
t192 = m_Hand.*t4.*t8.*t14.*t18.*t29.*2.4e+1;
t193 = m_Hand.*t4.*t8.*t14.*t18.*t31.*4.0e+1;
t194 = m_Hand.*t4.*t8.*t14.*t20.*t29.*4.0e+1;
t198 = (m_Hand.*t2.*t7.*t8.*t17.*t22)./6.0;
t199 = (m_Hand.*t4.*t7.*t8.*t10.*t17)./6.0;
t200 = (m_Hand.*t4.*t7.*t8.*t10.*t19)./6.0;
t202 = (m_Hand.*t5.*t7.*t8.*t11.*t18)./6.0;
t203 = -t167;
t204 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t8.*t11.*t27)./1.2e+1;
t205 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t8.*t11.*t29)./1.2e+1;
t207 = (m_Hand.*t2.*t6.*t8.*t22.*t29)./4.0;
t208 = (m_Hand.*t4.*t6.*t8.*t10.*t29)./4.0;
t209 = (m_Hand.*t4.*t6.*t8.*t10.*t31)./6.0;
t214 = -t172;
t221 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t8.*t11.*t16.*t28)./3.0;
t222 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t8.*t11.*t16.*t27)./6.0;
t223 = -t184;
t228 = -t190;
t230 = m_Hand.*t4.*t8.*t14.*t27.*t35.*2.0e+1;
t232 = m_Hand.*t8.*t9.*t15.*t25.*t27.*4.0;
t233 = m_Hand.*t8.*t11.*t17.*t23.*t27.*6.0;
t234 = m_Hand.*t8.*t13.*t19.*t21.*t27.*4.0;
t237 = (m_Hand.*t2.*t6.*t8.*t17.*t22)./1.2e+1;
t238 = (m_Hand.*t4.*t6.*t8.*t10.*t17)./1.2e+1;
t239 = (m_Hand.*t4.*t6.*t8.*t10.*t19)./1.2e+1;
t240 = -t197;
t242 = (m_Hand.*t3.*t6.*t8.*t11.*t29)./1.2e+1;
t243 = (m_Hand.*t3.*t6.*t8.*t11.*t27)./2.4e+1;
t244 = (m_Hand.*t3.*t7.*t8.*t11.*t28)./1.2e+1;
t245 = (m_Hand.*t5.*t7.*t8.*t11.*t16)./1.2e+1;
t247 = -t206;
t250 = (m_Hand.*t2.*t6.*t8.*t22.*t31)./1.2e+1;
t254 = -t211;
t255 = (m_Hand.*t3.*t6.*t8.*t23.*t27)./2.4e+1;
t256 = (m_Hand.*t3.*t7.*t8.*t23.*t28)./1.2e+1;
t258 = dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t8.*t23.*t27.*(-1.0./1.2e+1);
t260 = (m_Hand.*t6.*t8.*t11.*t16.*t27)./6.0;
t261 = (m_Hand.*t7.*t8.*t11.*t16.*t28)./3.0;
t262 = -t220;
t266 = m_Hand.*t8.*t11.*t15.*t21.*t27.*1.2e+1;
t267 = t29.*t124;
t269 = m_Hand.*t8.*t9.*t19.*t23.*t27.*1.2e+1;
t272 = m_Hand.*t8.*t13.*t17.*t21.*t29.*1.2e+1;
t277 = m_Hand.*t8.*t9.*t17.*t23.*t29.*2.4e+1;
t278 = m_Hand.*t8.*t11.*t15.*t21.*t31.*2.4e+1;
t280 = m_Hand.*t8.*t10.*t16.*t24.*t29.*3.6e+1;
t282 = m_Hand.*t8.*t12.*t16.*t22.*t29.*3.6e+1;
t284 = m_Hand.*t8.*t11.*t17.*t21.*t29.*4.8e+1;
t285 = m_Hand.*t8.*t12.*t16.*t22.*t31.*4.8e+1;
t287 = t21.*t140;
t288 = t15.*t147;
t291 = -t235;
t298 = -t251;
t299 = -t253;
t305 = m_Hand.*t8.*t13.*t15.*t21.*t31.*-1.2e+1;
t307 = -t275;
t309 = -t283;
t310 = m_Hand.*t8.*t15.*t63.*7.2e+1;
t311 = m_Hand.*t8.*t12.*t18.*t22.*t29.*7.2e+1;
t313 = t15.*t165;
t315 = (m_Hand.*t4.*t5.*t6.*t8.*t10.*t17)./2.4e+1;
t317 = -t289;
t318 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t8.*t63)./2.0;
t324 = t29.*t167;
t326 = t17.*t172;
t327 = t17.*t173;
t328 = -t303;
t331 = (m_Hand.*t3.*t6.*t8.*t63)./3.0;
t332 = t15.*t206;
t337 = (m_Hand.*t2.*t6.*t8.*t15.*t22.*t29)./6.0;
t341 = (m_Hand.*t4.*t7.*t8.*t10.*t17.*t27)./3.0;
t342 = t27.*t197;
t346 = (m_Hand.*t4.*t7.*t8.*t10.*t15.*t29)./6.0;
t347 = -t322;
t348 = dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t2.*t8.*t15.*t22.*t29.*(-1.0./3.0);
t349 = dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t8.*t9.*t21.*t29.*(-1.0./4.0);
t350 = dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t2.*t8.*t17.*t22.*t27.*(-1.0./6.0);
t351 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t8.*t16.*t63)./2.0;
t352 = -t333;
t353 = -t334;
t354 = -t335;
t356 = t29.*t235;
t361 = -t343;
t362 = -t344;
t363 = -t345;
t364 = (m_Hand.*t6.*t8.*t16.*t63)./4.0;
t368 = m_Hand.*t3.*t6.*t8.*t9.*t21.*t29.*(-1.0./8.0);
t369 = -t357;
t370 = m_Hand.*t2.*t6.*t8.*t17.*t22.*t27.*(-1.0./1.2e+1);
t372 = -t366;
t74 = m_Hand.*t8.*t27.*t37;
t82 = m_Hand.*t8.*t33.*t39;
t86 = -t73;
t88 = -t75;
t89 = -t76;
t91 = -t80;
t92 = m_Hand.*t8.*t11.*t34.*1.2e+1;
t110 = -t99;
t113 = t15.*t73;
t117 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t8.*t43)./6.0;
t122 = -t105;
t130 = m_Hand.*t8.*t27.*t33.*t34;
t131 = m_Hand.*t8.*t19.*t29.*t33.*4.0;
t133 = m_Hand.*t8.*t17.*t31.*t33.*6.0;
t136 = m_Hand.*t8.*t27.*t45.*2.4e+1;
t141 = -t112;
t142 = -t114;
t143 = -t115;
t144 = m_Hand.*t8.*t13.*t21.*t38.*4.0;
t146 = m_Hand.*t8.*t27.*t46.*3.6e+1;
t150 = (m_Hand.*t3.*t7.*t8.*t43)./6.0;
t154 = m_Hand.*t3.*t4.*t8.*t14.*t38.*4.0;
t161 = m_Hand.*t8.*t15.*t33.*t38.*4.0;
t168 = (dr_Beta_Hand.*dr_Alpha_Hand.*m_Hand.*t3.*t8.*t47)./6.0;
t178 = -t151;
t181 = -t156;
t201 = (m_Hand.*t3.*t7.*t8.*t47)./6.0;
t213 = -t171;
t215 = -t174;
t216 = t27.*t102;
t217 = -t177;
t218 = -t180;
t224 = -t185;
t225 = -t188;
t226 = m_Hand.*t3.*t8.*t12.*t22.*t38.*1.2e+1;
t227 = -t189;
t229 = m_Hand.*t4.*t8.*t14.*t16.*t38.*2.0e+1;
t231 = -t192;
t241 = -t199;
t246 = -t204;
t248 = -t208;
t257 = (m_Hand.*t7.*t8.*t16.*t43)./1.2e+1;
t263 = -t232;
t264 = -t233;
t265 = -t234;
t268 = m_Hand.*t8.*t9.*t31.*t46.*1.2e+1;
t270 = m_Hand.*t8.*t11.*t29.*t46.*1.2e+1;
t271 = t21.*t134;
t273 = m_Hand.*t8.*t9.*t27.*t46.*2.4e+1;
t274 = m_Hand.*t8.*t101.*2.4e+1;
t290 = t5.*t199;
t292 = -t237;
t293 = -t239;
t294 = -t243;
t295 = -t244;
t296 = -t245;
t297 = -t250;
t300 = -t255;
t301 = -t256;
t306 = -t272;
t308 = -t282;
t323 = t15.*t170;
t325 = t15.*t171;
t330 = -t315;
t336 = t15.*t208;
t355 = -t337;
t358 = t27.*t237;
t367 = -t351;
t371 = -t364;
t373 = t45+t46+t48+t50+t62+t69+t71+t112;
t94 = -t74;
t100 = -t82;
t162 = -t131;
t163 = -t133;
t164 = -t144;
t179 = -t154;
t195 = -t130;
t196 = -t161;
t279 = t21.*t136;
t302 = -t257;
t304 = -t270;
t374 = t43+t47+t58+t59+t70+t72+t77+t78+t88+t89+t90+t91+t97+t110;
t375 = t42+t49+t51+t53+t55+t57+t63+t64+t65+t67+t68+t86+t87+t98+t101+t113+t141;
t376 = t32+t60+t111+t116+t117+t118+t168+t169+t176+t198+t200+t207+t209+t217+t218+t221+t236+t238+t240+t241+t242+t247+t248+t249+t259+t260+t291+t292+t293+t294+t297+t300+t320+t321+t328+t331+t332+t336+t341+t342+t346+t354+t355+t363+t368+t370+t371;
t377 = t40+t41+t61+t93+t102+t139+t140+t148+t149+t150+t152+t165+t166+t170+t173+t175+t178+t181+t201+t202+t203+t205+t210+t212+t213+t214+t215+t219+t222+t246+t252+t254+t258+t261+t262+t288+t290+t295+t296+t298+t299+t301+t302+t312+t314+t316+t317+t318+t319+t325+t327+t329+t330+t338+t339+t340+t347+t348+t349+t350+t352+t353+t359+t360+t361+t362+t365+t367+t369+t372;
t378 = t81+t83+t84+t92+t94+t96+t100+t103+t106+t108+t109+t120+t121+t122+t123+t126+t127+t129+t132+t134+t135+t136+t137+t138+t142+t143+t145+t146+t153+t155+t157+t158+t159+t160+t162+t163+t164+t179+t182+t183+t186+t187+t191+t193+t194+t195+t196+t223+t224+t225+t226+t227+t228+t229+t230+t231+t263+t264+t265+t266+t267+t268+t269+t273+t274+t276+t277+t278+t279+t280+t281+t284+t285+t286+t304+t305+t306+t307+t308+t309+t310+t311;
t379 = 1.0./t378;
A11 = t8.*t15.*t373.*t379.*-4.8e+1;
if nargout > 1
    t380 = length_Hand.*t2.*t5.*t373.*t379.*4.8e+1;
    t381 = length_Hand.*t4.*t5.*t373.*t379.*4.8e+1;
    t383 = length_Hand.*t2.*t3.*t374.*t379.*2.4e+1;
    t384 = length_Hand.*t2.*t5.*t374.*t379.*2.4e+1;
    t385 = length_Hand.*t3.*t4.*t374.*t379.*2.4e+1;
    t386 = length_Hand.*t4.*t5.*t374.*t379.*2.4e+1;
    t388 = length_Hand.*t2.*t3.*t375.*t379.*4.8e+1;
    t389 = length_Hand.*t3.*t4.*t375.*t379.*4.8e+1;
    t390 = t374.*t376.*t379.*2.4e+1;
    t392 = t375.*t376.*t379.*4.8e+1;
    t393 = t373.*t377.*t379.*4.8e+1;
    t394 = t374.*t377.*t379.*2.4e+1;
    t382 = -t381;
    t387 = -t386;
    t391 = -t390;
    t395 = -t394;
    t396 = t380+t385;
    A12 = -length_Hand.*t3.*t396;
end
if nargout > 2
    A13 = -length_Hand.*t3.*(t381-t383);
end
if nargout > 3
    A14 = length_Hand.*t5.*t6-length_Hand.*t3.*(t390-t393);
end
if nargout > 4
    A21 = t4.*t8.*t15.*t374.*t379.*-2.4e+1-t2.*t3.*t5.*t8.*t373.*t379.*4.8e+1;
end
if nargout > 5
    t398 = t384+t389;
    A22 = -length_Hand.*t2.*t5.*t396-length_Hand.*t3.*t4.*t398;
end
if nargout > 6
    A23 = -length_Hand.*t2.*t5.*(t381-t383)-length_Hand.*t3.*t4.*(t386-t388);
end
if nargout > 7
    t397 = t382+t383;
    t399 = t387+t388;
    t400 = t391+t393;
    t401 = t392+t395;
    A24 = -length_Hand.*t2.*t3.*t6-length_Hand.*t2.*t3.*t7-length_Hand.*t3.*t4.*t401-length_Hand.*t2.*t5.*(t390-t393)+dr_Beta_Hand.*dr_Alpha_Hand.*length_Hand.*t4.*t5.*2.0;
end
if nargout > 8
    A31 = t2.*t8.*t15.*t374.*t379.*2.4e+1-t3.*t4.*t5.*t8.*t373.*t379.*4.8e+1;
end
if nargout > 9
    A32 = length_Hand.*t2.*t3.*t398-length_Hand.*t4.*t5.*t396;
end
if nargout > 10
    A33 = -length_Hand.*t4.*t5.*(t381-t383)+length_Hand.*t2.*t3.*(t386-t388);
end
if nargout > 11
    A34 = -length_Hand.*t3.*t4.*t6-length_Hand.*t3.*t4.*t7+length_Hand.*t2.*t3.*t401-length_Hand.*t4.*t5.*(t390-t393)-dr_Beta_Hand.*dr_Alpha_Hand.*length_Hand.*t2.*t5.*2.0;
end
