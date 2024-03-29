function ddr_Arm_Bottom = FRD_Ddr_Arm_Bottom(ddr_Beta_Hand,ddr_Alpha_Hand,dr_Beta_Hand,dr_Alpha_Hand,length_Hand,r_Alpha_Hand,r_Beta_Hand)
%FRD_DDR_ARM_BOTTOM
%    DDR_ARM_BOTTOM = FRD_DDR_ARM_BOTTOM(DDR_BETA_HAND,DDR_ALPHA_HAND,DR_BETA_HAND,DR_ALPHA_HAND,LENGTH_HAND,R_ALPHA_HAND,R_BETA_HAND)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    17-Jun-2021 17:17:08

t2 = cos(r_Alpha_Hand);
t3 = cos(r_Beta_Hand);
t4 = sin(r_Alpha_Hand);
t5 = sin(r_Beta_Hand);
t6 = dr_Beta_Hand.^2;
t7 = dr_Alpha_Hand.^2;
ddr_Arm_Bottom = [-ddr_Beta_Hand.*length_Hand.*t3+length_Hand.*t5.*t6,-ddr_Beta_Hand.*length_Hand.*t2.*t5-ddr_Alpha_Hand.*length_Hand.*t3.*t4-length_Hand.*t2.*t3.*t6-length_Hand.*t2.*t3.*t7+dr_Beta_Hand.*dr_Alpha_Hand.*length_Hand.*t4.*t5.*2.0,-ddr_Beta_Hand.*length_Hand.*t4.*t5+ddr_Alpha_Hand.*length_Hand.*t2.*t3-length_Hand.*t3.*t4.*t6-length_Hand.*t3.*t4.*t7-dr_Beta_Hand.*dr_Alpha_Hand.*length_Hand.*t2.*t5.*2.0];
