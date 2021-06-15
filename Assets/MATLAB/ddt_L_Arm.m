function dotq = ddt_L_Arm(~, q, g, length_Hand, m_Hand, l_Tau_Beta_Shoulder, l_Tau_Alpha_Shoulder, l_F_X, l_F_Y, l_F_Z)
%DDT_L_ARM この関数の概要をここに記述
%   詳細説明をここに記述

l_Alpha_Hand = q(1);
dl_Alpha_Hand = q(2);
l_Beta_Hand = q(3);
dl_Beta_Hand = q(4);

[ddl_Alpha_Hand,ddl_Beta_Hand] = FFD_Dds_Arm_L(dl_Beta_Hand,dl_Alpha_Hand,g,l_Alpha_Hand,l_Beta_Hand,l_F_X,l_F_Y,l_F_Z,l_Tau_Beta_Shoulder,l_Tau_Alpha_Shoulder,length_Hand,m_Hand);
% [ddl_Alpha_Hand,ddl_Beta_Hand] = FFD_Dds_Arm_L(dl_Beta_Hand,dl_Alpha_Hand,g,l_Alpha_Hand,l_Beta_Hand,l_F_X,l_F_Y,l_F_Z,length_Hand,m_Hand);

dotq = [dl_Alpha_Hand, ddl_Alpha_Hand, dl_Beta_Hand, ddl_Beta_Hand]';
end

