function dotq = ddt_HFD(t,q,r_P_Fixed,l_P_Fixed, g, length_Hand, m_Hand, m_Body, width_Body, height_Body, depth_Body)

r_X_Fixed = r_P_Fixed(1);
r_Y_Fixed = r_P_Fixed(2);
r_Z_Fixed = r_P_Fixed(3);

l_X_Fixed = l_P_Fixed(1);
l_Y_Fixed = l_P_Fixed(2);
l_Z_Fixed = l_P_Fixed(3);

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

dr_Arm_Bottom = FRD_Dr_Arm_Bottom(dr_Beta_Hand,dr_Alpha_Hand,length_Hand,r_Alpha_Hand,r_Beta_Hand);
dl_Arm_Bottom = FRD_Dl_Arm_Bottom(dl_Beta_Hand,dl_Alpha_Hand,l_Alpha_Hand,l_Beta_Hand,length_Hand);

dx_Head = (dl_Arm_Bottom(1) + dr_Arm_Bottom(1))/2;
dy_Head = (dl_Arm_Bottom(2) + dr_Arm_Bottom(2))/2;
dz_Head = (dl_Arm_Bottom(3) + dr_Arm_Bottom(3))/2;

ddbeta_Body = -2 * dbeta_Body - beta_Body;
ddgamma_Body = -2 * dgamma_Body - gamma_Body;

%% r_Arm_Bottom
[A11,A12,A13,A21,A22,A23,A31,A32,A33] = HFD_Coeffs_Ddr_Arm_Bottom_Force(length_Hand,m_Hand,r_Alpha_Hand,r_Beta_Hand);
coeffs_Ddr_Arm_Bottom_Force = [
    A11,A12,A13,0,0,0;
    A21,A22,A23,0,0,0;
    A31,A32,A33,0,0,0;
    ];
[A11,A12,A21,A22,A31,A32] = HFD_Coeffs_Ddr_Arm_Bottom_Tau(length_Hand,m_Hand,r_Alpha_Hand,r_Beta_Hand);
coeffs_Ddr_Arm_Bottom_Tau = [
    A11, A12,;
    A21, A22,;
    A31, A32,;
    ];
[A11,A21,A31] = HFD_Coeffs_Ddr_Arm_Bottom_Constant(dr_Beta_Hand,dr_Alpha_Hand,g,length_Hand,m_Hand,r_Alpha_Hand,r_Beta_Hand);
coeffs_Ddr_Arm_Bottom_Constant = [
    A11;
    A21;
    A31;
    ];

%% l_Arm_Bottom
[A11,A12,A13,A21,A22,A23,A31,A32,A33] = HFD_Coeffs_Ddl_Arm_Bottom_Force(l_Alpha_Hand,l_Beta_Hand,length_Hand,m_Hand);
coeffs_Ddl_Arm_Bottom_Force = [
    A11,A12,A13,0,0,0;
    A21,A22,A23,0,0,0;
    A31,A32,A33,0,0,0;
    ];
[A11,A12,A21,A22,A31,A32] = HFD_Coeffs_Ddl_Arm_Bottom_Tau(l_Alpha_Hand,l_Beta_Hand,length_Hand,m_Hand);
coeffs_Ddl_Arm_Bottom_Tau = [
    A11, A12,;
    A21, A22,;
    A31, A32,;
    ];
[A11,A21,A31] = HFD_Coeffs_Ddl_Arm_Bottom_Constant(dl_Beta_Hand,dl_Alpha_Hand,g,l_Alpha_Hand,l_Beta_Hand,length_Hand,m_Hand);
coeffs_Ddl_Arm_Bottom_Constant = [
    A11;
    A21;
    A31;
    ];

%% body
[A11,A12,A13,A14,A15,A16] = HFD_Coeffs_Ddr_Shoulder_Force_Row1(m_Body);
[A21,A22,A23,A24,A25,A26] = HFD_Coeffs_Ddr_Shoulder_Force_Row2(alpha_Body,beta_Body,depth_Body,gamma_Body,height_Body,l_Alpha_Hand,m_Body,r_Alpha_Hand,width_Body);
[A31,A32,A33,A34,A35,A36] = HFD_Coeffs_Ddr_Shoulder_Force_Row3(alpha_Body,beta_Body,depth_Body,gamma_Body,height_Body,l_Alpha_Hand,m_Body,r_Alpha_Hand,width_Body);
coeffs_Ddr_Shoulder_Force = [
    A11, A12, A13, A14, A15, A16;
    A21, A22, A23, A24, A25, A26;
    A31, A32, A33, A34, A35, A36;
    ];
[A11,A21,A31] = HFD_Coeffs_Ddr_Shoulder_Constant(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,ddbeta_Body,ddgamma_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_Alpha_Hand,m_Body,r_Alpha_Hand,width_Body);
coeffs_Ddr_Shoulder_Constant = [A11,A21,A31]';

[A11,A12,A13,A14,A15,A16] = HFD_Coeffs_Ddl_Shoulder_Force_Row1(m_Body);
[A21,A22,A23,A24,A25,A26] = HFD_Coeffs_Ddl_Shoulder_Force_Row2(alpha_Body,beta_Body,depth_Body,gamma_Body,height_Body,l_Alpha_Hand,m_Body,r_Alpha_Hand,width_Body);
[A31,A32,A33,A34,A35,A36] = HFD_Coeffs_Ddl_Shoulder_Force_Row3(alpha_Body,beta_Body,depth_Body,gamma_Body,height_Body,l_Alpha_Hand,m_Body,r_Alpha_Hand,width_Body);
coeffs_Ddl_Shoulder_Force = [
    A11, A12, A13, A14, A15, A16, ;
    A21, A22, A23, A24, A25, A26, ;
    A31, A32, A33, A34, A35, A36, ;
    ];
[A11,A21,A31] = HFD_Coeffs_Ddl_Shoulder_Constant(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,ddbeta_Body,ddgamma_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_Alpha_Hand,m_Body,r_Alpha_Hand,width_Body);
coeffs_Ddl_Shoulder_Constant = [A11,A21,A31]';

[A11,A12,A13,A14,A15,A16] = HFD_Coeffs_R_Tau_Alpba_Shoulder_Force(alpha_Body,beta_Body,depth_Body,gamma_Body,height_Body,l_Alpha_Hand,r_Alpha_Hand,width_Body);
coeffs_R_Tau_Alpha_Shoulder_Force = [
    A11, A12, A13, A14, A15, A16, ;
    ];
A17 = HFD_Coeffs_R_Tau_Alpba_Shoulder_Constant(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,ddbeta_Body,ddgamma_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_Alpha_Hand,m_Body,r_Alpha_Hand,width_Body);
coeffs_R_Tau_Alpha_Shoulder_Constant = A17;

[A11,A12,A13,A14,A15,A16] = HFD_Coeffs_R_Tau_Beta_Shoulder_Force(alpha_Body,beta_Body,depth_Body,gamma_Body,height_Body,l_Alpha_Hand,r_Alpha_Hand,width_Body);
coeffs_R_Tau_Beta_Shoulder_Force = [
    A11, A12, A13, A14, A15, A16, ;
    ];
A17 = HFD_Coeffs_R_Tau_Beta_Shoulder_Constant(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,ddbeta_Body,ddgamma_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_Alpha_Hand,m_Body,r_Alpha_Hand,width_Body);
coeffs_R_Tau_Beta_Shoulder_Constant = A17;

%{
%% arm_R_Bottom
[A11,A12,A13,A21,A22,A23,A31,A32,A33] = HFD_Coeffs_Ddr_Arm_Bottom_Force(length_Hand,m_Hand,r_Alpha_Hand,r_Beta_Hand);
coeffs_Ddr_Arm_Bottom_Force = [
    A11, A12, A13, 0, 0, 0;
    A21, A22, A23, 0, 0, 0;
    A31, A32, A33, 0, 0, 0;
    ];
[A11,A12,A13] = HFD_Coeffs_R_Tau_Beta_Shoulder_Force(length_Hand,r_Alpha_Hand,r_Beta_Hand);
coeffs_R_Tau_Beta_Shoulder_Force = [A11, A12, A13, 0, 0, 0];
[A11,A21,A31] = HFD_Coeffs_Ddr_Arm_Bottom_Constant(ddr_Beta_Hand,dr_Beta_Hand,dr_Alpha_Hand,g,length_Hand,m_Hand,r_Alpha_Hand,r_Beta_Hand,r_Tau_Alpha_Shoulder);
coeffs_Ddr_Arm_Bottom_Constant = [A11, A21, A31]';

%% arm_L_Bottom
[A11,A12,A13,A21,A22,A23,A31,A32,A33] = HFD_Coeffs_Ddl_Arm_Bottom_Force(l_Alpha_Hand,l_Beta_Hand,length_Hand,m_Hand);
coeffs_Ddl_Arm_Bottom_Force = [
    0, 0, 0, A11, A12, A13, ;
    0, 0, 0, A21, A22, A23, ;
    0, 0, 0, A31, A32, A33, ;
    ];
[A11,A12,A13] = HFD_Coeffs_L_Tau_Beta_Shoulder_Force(l_Alpha_Hand,l_Beta_Hand,length_Hand);
coeffs_L_Tau_Beta_Shoulder_Force = [0, 0, 0, A11, A12, A13];
[A11,A21,A31] = HFD_Coeffs_Ddl_Arm_Bottom_Constant(ddl_Beta_Hand,dl_Beta_Hand,dl_Alpha_Hand,g,l_Alpha_Hand,l_Beta_Hand,l_Tau_Alpha_Shoulder,length_Hand,m_Hand);
coeffs_Ddl_Arm_Bottom_Constant = [A11, A21, A31]';

%% shoulder_R
[A11,A12,A13,A14,A15,A16,A21,A22,A23,A24,A25,A26,A31,A32,A33,A34,A35,A36] = HFD_Coeffs_Ddr_Shoulder_Force(alpha_Body,beta_Body,depth_Body,gamma_Body,height_Body,m_Body,width_Body);
coeffs_Ddr_Shoulder_Force = [
    A11, A12, A13, A14, A15, A16;
    A21, A22, A23, A24, A25, A26;
    A31, A32, A33, A34, A35, A36;
    ];
[A11,A12,A21,A22,A31,A32] = HFD_Coeffs_Ddr_Shoulder_Tau_Beta(alpha_Body,beta_Body,depth_Body,gamma_Body,height_Body,l_Alpha_Hand,m_Body,r_Alpha_Hand,width_Body);
coeffs_Ddr_Shoulder_Tau_Beta = [
    A11, A12, ;
    A21, A22, ;
    A31, A32, ;
    ];
[A11,A21,A31] = HFD_Coeffs_Ddr_Shoulder_Constant(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_Tau_Alpha_Shoulder,m_Body,r_Tau_Alpha_Shoulder,width_Body);
coeffs_Ddr_Shoulder_Constant = [A11,A21,A31]';

%% shoulder_L
[A11,A12,A13,A14,A15,A16,A21,A22,A23,A24,A25,A26,A31,A32,A33,A34,A35,A36] = HFD_Coeffs_Ddl_Shoulder_Force(alpha_Body,beta_Body,depth_Body,gamma_Body,height_Body,m_Body,width_Body);
coeffs_Ddl_Shoulder_Force = [
    A11, A12, A13, A14, A15, A16, ;
    A21, A22, A23, A24, A25, A26, ;
    A31, A32, A33, A34, A35, A36, ;
    ];
[A11,A12,A21,A22,A31,A32] = HFD_Coeffs_Ddl_Shoulder_Tau_Beta(alpha_Body,beta_Body,depth_Body,gamma_Body,height_Body,l_Alpha_Hand,m_Body,r_Alpha_Hand,width_Body);
coeffs_Ddl_Shoulder_Tau_Beta = [
    A11, A12, ;
    A21, A22, ;
    A31, A32, ;
    ];
[A11,A21,A31] = HFD_Coeffs_Ddl_Shoulder_Constant(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_Tau_Alpha_Shoulder,m_Body,r_Tau_Alpha_Shoulder,width_Body);
coeffs_Ddl_Shoulder_Constant = [A11,A21,A31]';
%}

%%
coeffs_Matrix_Force = [
    coeffs_Ddr_Arm_Bottom_Force
    coeffs_Ddl_Arm_Bottom_Force
    ] + [
    coeffs_Ddr_Arm_Bottom_Tau * [coeffs_R_Tau_Alpha_Shoulder_Force; coeffs_R_Tau_Beta_Shoulder_Force]
    coeffs_Ddl_Arm_Bottom_Tau * [coeffs_R_Tau_Alpha_Shoulder_Force; coeffs_R_Tau_Beta_Shoulder_Force]
    ] - [
    coeffs_Ddr_Shoulder_Force
    coeffs_Ddl_Shoulder_Force
    ];

coeffs_Target = [
    coeffs_Ddr_Arm_Bottom_Constant
    coeffs_Ddl_Arm_Bottom_Constant
    ] + [
    coeffs_Ddr_Arm_Bottom_Tau * [coeffs_R_Tau_Alpha_Shoulder_Constant; coeffs_R_Tau_Beta_Shoulder_Constant]
    coeffs_Ddr_Arm_Bottom_Tau * [coeffs_R_Tau_Alpha_Shoulder_Constant; coeffs_R_Tau_Beta_Shoulder_Constant]
    ] - [
    coeffs_Ddr_Shoulder_Constant
    coeffs_Ddl_Shoulder_Constant
    ];

f_All = inv(coeffs_Matrix_Force) * (-coeffs_Target);

r_F_X = f_All(1);
r_F_Y = f_All(2);
r_F_Z = f_All(3);

l_F_X = f_All(4);
l_F_Y = f_All(5);
l_F_Z = f_All(6);

[ddalpha_Body,ddx_Head,ddy_Head,ddz_Head,r_Tau_Alpha_Shoulder,r_Tau_Beta_Shoulder] = HFD_Dds_Body(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,ddbeta_Body,ddgamma_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_Alpha_Hand,l_F_X,l_F_Y,l_F_Z,m_Body,r_Alpha_Hand,r_F_X,r_F_Y,r_F_Z,width_Body);
l_Tau_Alpha_Shoulder = r_Tau_Alpha_Shoulder;
l_Tau_Beta_Shoulder = r_Tau_Beta_Shoulder;

[ddr_Alpha_Hand,ddr_Beta_Hand] = FFD_Dds_Arm_R(dr_Beta_Hand,dr_Alpha_Hand,g,length_Hand,m_Hand,r_Alpha_Hand,r_Beta_Hand,r_F_X,r_F_Y,r_F_Z,r_Tau_Beta_Shoulder,r_Tau_Alpha_Shoulder);
[ddl_Alpha_Hand,ddl_Beta_Hand] = FFD_Dds_Arm_L(dl_Beta_Hand,dl_Alpha_Hand,g,l_Alpha_Hand,l_Beta_Hand,l_F_X,l_F_Y,l_F_Z,l_Tau_Beta_Shoulder,l_Tau_Alpha_Shoulder,length_Hand,m_Hand);

% ddr_Arm_Bottom = find_Ddr_Arm_Bottom(dr_Beta_Hand,dr_Alpha_Hand,g,length_Hand,m_Hand,r_Alpha_Hand,r_Beta_Hand,r_F_X,r_F_Y,r_F_Z);
% ddl_Arm_Bottom = find_Ddl_Arm_Bottom(dl_Beta_Hand,dl_Alpha_Hand,g,l_Alpha_Hand,l_Beta_Hand,l_F_X,l_F_Y,l_F_Z,length_Hand,m_Hand);
% ddr_Shoulder = find_Ddr_Shoulder(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_F_X,l_F_Y,l_F_Z,m_Body,r_F_X,r_F_Y,r_F_Z,width_Body);
% ddl_Shoulder = find_Ddl_Shoulder(alpha_Body,beta_Body,dalpha_Body,dbeta_Body,depth_Body,dgamma_Body,g,gamma_Body,height_Body,l_F_X,l_F_Y,l_F_Z,m_Body,r_F_X,r_F_Y,r_F_Z,width_Body);
% 
% [ddr_Arm_Bottom - ddr_Shoulder, ddl_Arm_Bottom - ddl_Shoulder]

dd_Threshold = 0;
if abs(ddbeta_Body) < dd_Threshold
    if (dbeta_Body == 0)
        ddbeta_Body = 0;
    end
end
if abs(ddgamma_Body) < dd_Threshold
    if (dgamma_Body == 0)
        ddgamma_Body = 0;
    end
end

if abs(ddr_Beta_Hand) < dd_Threshold
    if (dr_Beta_Hand == 0)
        ddr_Beta_Hand = 0;
    end
end
if abs(ddl_Beta_Hand) < dd_Threshold
    if (dl_Beta_Hand == 0)
        ddl_Beta_Hand = 0;
    end
end

dotq = [dr_Alpha_Hand, ddr_Alpha_Hand, dr_Beta_Hand, ddr_Beta_Hand, dl_Alpha_Hand, ddl_Alpha_Hand, dl_Beta_Hand, ddl_Beta_Hand, ...
    dalpha_Body, ddalpha_Body, dbeta_Body, ddbeta_Body, dgamma_Body, ddgamma_Body, ...
    dx_Head, ddx_Head, dy_Head, ddy_Head, dz_Head, ddz_Head]';

end















































