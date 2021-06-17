function dotq = ddt_Arms(t, q,g,length_Hand,m_Hand,radius_Hand,k, r_P_Fixed, l_P_Fixed, r_Tau_Alpha_Shoulder, r_Tau_Beta_Shoulder, r_Tau_Gamma_Shoulder, l_Tau_Alpha_Shoulder, l_Tau_Beta_Shoulder, l_Tau_Gamma_Shoulder)
%DDT_ARMS この関数の概要をここに記述
%   詳細説明をここに記述

r_Alpha_Hand = q(1);
dr_Alpha_Hand = q(2);
r_Beta_Hand = q(3);
dr_Beta_Hand = q(4);
r_Gamma_Hand = q(5);
dr_Gamma_Hand = q(6);

l_Alpha_Hand = q(7);
dl_Alpha_Hand = q(8);
l_Beta_Hand = q(9);
dl_Beta_Hand = q(10);
l_Gamma_Hand = q(11);
dl_Gamma_Hand = q(12);

r_X_Fixed = r_P_Fixed(1);
r_Y_Fixed = r_P_Fixed(2);
r_Z_Fixed = r_P_Fixed(3);

l_X_Fixed = l_P_Fixed(1);
l_Y_Fixed = l_P_Fixed(2);
l_Z_Fixed = l_P_Fixed(3);

r_Arm_Bottom = FRD_R_Arm_Bottom(length_Hand,r_Alpha_Hand,r_Beta_Hand,r_X_Fixed,r_Y_Fixed,r_Z_Fixed);
l_Arm_Bottom = FRD_L_Arm_Bottom(l_Alpha_Hand,l_Beta_Hand,l_X_Fixed,l_Y_Fixed,l_Z_Fixed,length_Hand);

F_All = -k * (r_Arm_Bottom - l_Arm_Bottom - (r_P_Fixed - l_P_Fixed));

r_F_X = F_All(1);
r_F_Y = F_All(2);
r_F_Z = F_All(3);

l_F_X = -F_All(1);
l_F_Y = -F_All(2);
l_F_Z = -F_All(3);

[ddr_Alpha_Hand,ddr_Beta_Hand,ddr_Gamma_Hand] = FFD_Dds_Arm_R(dr_Beta_Hand,dr_Gamma_Hand,dr_Alpha_Hand,g,length_Hand,m_Hand,r_Alpha_Hand,r_Beta_Hand,r_F_X,r_F_Y,r_F_Z,r_Gamma_Hand,r_Tau_Beta_Shoulder,r_Tau_Gamma_Shoulder,r_Tau_Alpha_Shoulder,radius_Hand);
[ddl_Alpha_Hand,ddl_Beta_Hand,ddl_Gamma_Hand] = FFD_Dds_Arm_L(dl_Beta_Hand,dl_Gamma_Hand,dl_Alpha_Hand,g,l_Alpha_Hand,l_Beta_Hand,l_F_X,l_F_Y,l_F_Z,l_Gamma_Hand,l_Tau_Beta_Shoulder,l_Tau_Gamma_Shoulder,l_Tau_Alpha_Shoulder,length_Hand,m_Hand,radius_Hand);

dotq = [dr_Alpha_Hand, ddr_Alpha_Hand, dr_Beta_Hand, ddr_Beta_Hand, dr_Gamma_Hand, ddr_Gamma_Hand, ...
    dl_Alpha_Hand, ddl_Alpha_Hand, dl_Beta_Hand, ddl_Beta_Hand, dl_Gamma_Hand, ddl_Gamma_Hand]';
end

