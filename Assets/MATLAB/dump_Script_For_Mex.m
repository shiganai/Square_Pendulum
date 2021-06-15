
clear all

width_Body = 1;
height_Body = 2;
depth_Body = 0.5;
m_Body = 10;
m_Hand = 1;
length_Hand = 1;
g = 1;

alpha_Body = deg2rad(0);
beta_Body = deg2rad(0);
gamma_Body = deg2rad(0);
x_Head = 0;
y_Head = 1;
z_Head = 0;

r_Alpha_Hand = deg2rad(-0);
r_Beta_Hand = deg2rad(0);

l_Alpha_Hand = deg2rad(-0);
l_Beta_Hand = deg2rad(0);

tau_Alpha_Body = 0;

%%

r_P_Fixed = [0,0,0];
l_P_Fixed = [0, 0, 0];

r_X_Fixed = r_P_Fixed(1);
r_Y_Fixed = r_P_Fixed(2);
r_Z_Fixed = r_P_Fixed(3);

l_X_Fixed = l_P_Fixed(1);
l_Y_Fixed = l_P_Fixed(2);
l_Z_Fixed = l_P_Fixed(3);

r_Arm_Bottom = FRD_R_Arm_Bottom(length_Hand,r_Alpha_Hand,r_Beta_Hand,r_X_Fixed,r_Y_Fixed,r_Z_Fixed);
l_Arm_Bottom = FRD_L_Arm_Bottom(l_Alpha_Hand,l_Beta_Hand,l_X_Fixed,l_Y_Fixed,l_Z_Fixed,length_Hand);

r_Shoulder = FRD_R_Shoulder(alpha_Body,beta_Body,gamma_Body,width_Body,x_Head,y_Head,z_Head);
l_Shoulder = FRD_L_Shoulder(alpha_Body,beta_Body,gamma_Body,width_Body,x_Head,y_Head,z_Head);

%%
r_P_Fixed = r_P_Fixed + (r_Shoulder - r_Arm_Bottom);
l_P_Fixed = l_P_Fixed + (l_Shoulder - l_Arm_Bottom);

r_X_Fixed = r_P_Fixed(1);
r_Y_Fixed = r_P_Fixed(2);
r_Z_Fixed = r_P_Fixed(3);

l_X_Fixed = l_P_Fixed(1);
l_Y_Fixed = l_P_Fixed(2);
l_Z_Fixed = l_P_Fixed(3);

q = [r_Alpha_Hand, 0, r_Beta_Hand, 0, l_Alpha_Hand, 0, l_Beta_Hand, 0, ...
    alpha_Body, 0, beta_Body, 0, gamma_Body, 0, ...
    x_Head, 0, y_Head, 0, z_Head, 0]';

r_Alpha_Hand = q(1);
dr_Alpha_Hand = q(2);

r_Beta_Hand = q(3);
dr_Beta_Hand = q(4);

l_Alpha_Hand = q(5);
dl_Alpha_Hand = q(6);

l_Beta_Hand = q(7);
dl_Beta_Hand = q(8);

alpha_Body = q(9);
dalpha_Body = q(10);

beta_Body = q(11);
dbeta_Body = q(12);

gamma_Body = q(13);
dgamma_Body = q(14);

% x_Head = q(15);
dx_Head = q(16);
% 
% y_Head = q(17);
dy_Head = q(18);
% 
% z_Head = q(19);
dz_Head = q(20);

r_Tau_Alpha_Shoulder = -0.;
l_Tau_Alpha_Shoulder = -0.;

r_Tau_Beta_Shoulder = -0.;
l_Tau_Beta_Shoulder = -0.;

tic
[A11,A12,A13,A14,A15,A16,A17,A21,A22,A23,A24,A25,A26,A27,A31,A32,A33,A34,A35,A36,A37] = FFD_Coeffs_Ddr_Shoulder(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_Alpha_Hand,l_Tau_Beta_Shoulder,l_Tau_Alpha_Shoulder,m_Body,r_Alpha_Hand,r_Tau_Beta_Shoulder,r_Tau_Alpha_Shoulder,width_Body);
coeffs_Ddr_Shoulder = [
    A11,A12,A13,A14,A15,A16,A17,;
    A21,A22,A23,A24,A25,A26,A27,;
    A31,A32,A33,A34,A35,A36,A37
    ];
toc

% [A11,A12,A13,A14,A15,A16,A17,A21,A22,A23,A24,A25,A26,A27,A31,A32,A33,A34,A35,A36,A37] = FFD_Coeffs_Ddl_Shoulder(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_Alpha_Hand,l_Tau_Beta_Shoulder,l_Tau_Alpha_Shoulder,m_Body,r_Alpha_Hand,r_Tau_Beta_Shoulder,r_Tau_Alpha_Shoulder,width_Body);
% coeffs_Ddl_Shoulder = [
%     A11,A12,A13,A14,A15,A16,A17,;
%     A21,A22,A23,A24,A25,A26,A27,;
%     A31,A32,A33,A34,A35,A36,A37
%     ];
% 
% f_All = zeros(6,1);
% 
% r_F_X = f_All(1);
% r_F_Y = f_All(2);
% r_F_Z = f_All(3);
% 
% l_F_X = f_All(4);
% l_F_Y = f_All(5);
% l_F_Z = f_All(6);
% 
% [ddalpha_Body,ddbeta_Body,ddgamma_Body,ddx_Head,ddy_Head,ddz_Head] = FFD_Dds_Body(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_Alpha_Hand,l_F_X,l_F_Y,l_F_Z,l_Tau_Beta_Shoulder,l_Tau_Alpha_Shoulder,m_Body,r_Alpha_Hand,r_F_X,r_F_Y,r_F_Z,r_Tau_Beta_Shoulder,r_Tau_Alpha_Shoulder,width_Body);
% 
