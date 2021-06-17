
clear all

m_Hand = 1;
length_Hand = 1;
radius_Hand = 1;
g = 1;
k = 1;

width_Body = 1;

l_Alpha_Hand = deg2rad(0);
l_Beta_Hand = deg2rad(0);
l_Gamma_Hand = deg2rad(0);

r_Alpha_Hand = deg2rad(0);
r_Beta_Hand = deg2rad(0);
r_Gamma_Hand = deg2rad(0);

r_P_Fixed = [0,0,0];
l_P_Fixed = r_P_Fixed + [0,length_Hand,0];

r_X_Fixed = r_P_Fixed(1);
r_Y_Fixed = r_P_Fixed(2);
r_Z_Fixed = r_P_Fixed(3);

l_X_Fixed = l_P_Fixed(1);
l_Y_Fixed = l_P_Fixed(2);
l_Z_Fixed = l_P_Fixed(3);

r_Tau_Alpha_Shoulder = 0;
r_Tau_Beta_Shoulder = 0;
r_Tau_Gamma_Shoulder = 0;

l_Tau_Alpha_Shoulder = 0;
l_Tau_Beta_Shoulder = 0;
l_Tau_Gamma_Shoulder = 0;

time = 0:1e-2:10;
q = [r_Alpha_Hand, 0, r_Beta_Hand, 0, r_Gamma_Hand, 0, ...
    l_Alpha_Hand, 0, l_Beta_Hand, 0, l_Gamma_Hand, 0]';

[time, q] = ode45(@(t,q) ddt_Arms(t, q,g,length_Hand,m_Hand,radius_Hand,k, r_P_Fixed, l_P_Fixed, ...
    r_Tau_Alpha_Shoulder, r_Tau_Beta_Shoulder, r_Tau_Gamma_Shoulder, l_Tau_Alpha_Shoulder, l_Tau_Beta_Shoulder, l_Tau_Gamma_Shoulder), ...
    time, q);

r_Alpha_Hand = q(:, 1);
dr_Alpha_Hand = q(:, 2);
r_Beta_Hand = q(:, 3);
dr_Beta_Hand = q(:, 4);
r_Gamma_Hand = q(:, 5);
dr_Gamma_Hand = q(:, 6);

l_Alpha_Hand = q(:, 7);
dl_Alpha_Hand = q(:, 8);
l_Beta_Hand = q(:, 9);
dl_Beta_Hand = q(:, 10);
l_Gamma_Hand = q(:, 11);
dl_Gamma_Hand = q(:, 12);

r_Arm_Bottom = FRD_R_Arm_Bottom(length_Hand,r_Alpha_Hand,r_Beta_Hand,r_X_Fixed,r_Y_Fixed,r_Z_Fixed);
l_Arm_Bottom = FRD_L_Arm_Bottom(l_Alpha_Hand,l_Beta_Hand,l_X_Fixed,l_Y_Fixed,l_Z_Fixed,length_Hand);

zero_Array = zeros(size(r_Arm_Bottom, 1));
nan_Array = nan(size(r_Arm_Bottom, 1));

x_Array = [r_X_Fixed + zero_Array, r_Arm_Bottom(1), nan_Array, l_X_Fixed + zero_Array, l_Arm_Bottom(1)];
y_Array = [r_Y_Fixed + zero_Array, r_Arm_Bottom(2), nan_Array, l_Y_Fixed + zero_Array, l_Arm_Bottom(2)];
z_Array = [r_Z_Fixed + zero_Array, r_Arm_Bottom(3), nan_Array, l_Z_Fixed + zero_Array, l_Arm_Bottom(3)];





































