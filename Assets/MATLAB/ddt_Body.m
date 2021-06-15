function dotq = ddt_Body(t, q, g, m_Body, depth_Body, height_Body, width_Body, r_Alpha_Hand, l_Alpha_Hand, r_Tau_Beta_Shoulder, r_Tau_Alpha_Shoulder, l_Tau_Beta_Shoulder, l_Tau_Alpha_Shoulder, r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z)
%DDT_BODY この関数の概要をここに記述
%   詳細説明をここに記述

alpha_Body = q(1);
dalpha_Body = q(2);
beta_Body = q(3);
dbeta_Body = q(4);
gamma_Body = q(5);
dgamma_Body = q(6);
x_Head = q(7);
dx_Head = q(8);
y_Head = q(9);
dy_Head = q(10);
z_Head = q(11);
dz_Head = q(12);

[ddalpha_Body,ddbeta_Body,ddgamma_Body,ddx_Head,ddy_Head,ddz_Head] = ...
    FFD_Dds_Body(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_Alpha_Hand,l_F_X,l_F_Y,l_F_Z,l_Tau_Beta_Shoulder,l_Tau_Alpha_Shoulder,m_Body,r_Alpha_Hand,r_F_X,r_F_Y,r_F_Z,r_Tau_Beta_Shoulder,r_Tau_Alpha_Shoulder,width_Body);


dotq = [dalpha_Body, ddalpha_Body, dbeta_Body, ddbeta_Body, dgamma_Body, ddgamma_Body, dx_Head, ddx_Head, dy_Head, ddy_Head, dz_Head, ddz_Head]';
end

