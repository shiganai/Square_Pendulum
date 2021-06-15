
clear all

width_Body = 1;
height_Body = 2;
depth_Body = 0.5;
m_Body = 10;
m_Hand = 1;
length_Hand = 1;
g = 1;

alpha_Body = deg2rad(0);
beta_Body = deg2rad(10);
gamma_Body = deg2rad(0);
x_Head = 0;
y_Head = 1;
z_Head = 0;

r_Alpha_Hand = deg2rad(0);
r_Beta_Hand = deg2rad(0);

l_Alpha_Hand = deg2rad(0);
l_Beta_Hand = deg2rad(0);

tau_Alpha_Body = 0;
r_Tau_Alpha_Hand = 10;
l_Tau_Alpha_Hand = 10;

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

r_Hip = FRD_R_Hip(alpha_Body,beta_Body,gamma_Body,height_Body,width_Body,x_Head,y_Head,z_Head);
l_Hip = FRD_L_Hip(alpha_Body,beta_Body,gamma_Body,height_Body,width_Body,x_Head,y_Head,z_Head);

%%
r_P_Fixed = r_P_Fixed + (r_Shoulder - r_Arm_Bottom);
l_P_Fixed = l_P_Fixed + (l_Shoulder - l_Arm_Bottom);

r_X_Fixed = r_P_Fixed(1);
r_Y_Fixed = r_P_Fixed(2);
r_Z_Fixed = r_P_Fixed(3);

l_X_Fixed = l_P_Fixed(1);
l_Y_Fixed = l_P_Fixed(2);
l_Z_Fixed = l_P_Fixed(3);


r_Arm_Bottom = FRD_R_Arm_Bottom(length_Hand,r_Alpha_Hand,r_Beta_Hand,r_X_Fixed,r_Y_Fixed,r_Z_Fixed);
l_Arm_Bottom = FRD_L_Arm_Bottom(l_Alpha_Hand,l_Beta_Hand,l_X_Fixed,l_Y_Fixed,l_Z_Fixed,length_Hand);

%%
nan_Array = nan(size(l_Shoulder, 1), 1);
zero_Array = zeros(size(l_Shoulder, 1), 1);

x_Array = [r_Shoulder(:, 1), l_Shoulder(:, 1), l_Hip(:, 1), r_Hip(:, 1), r_Shoulder(:, 1), ...
    nan_Array, zero_Array + l_P_Fixed(1), l_Arm_Bottom(:, 1), nan_Array, zero_Array + r_P_Fixed(1), r_Arm_Bottom(:, 1)];
y_Array = [r_Shoulder(:, 2), l_Shoulder(:, 2), l_Hip(:, 2), r_Hip(:, 2), r_Shoulder(:, 2), ...
    nan_Array, zero_Array + l_P_Fixed(2), l_Arm_Bottom(:, 2), nan_Array, zero_Array + r_P_Fixed(2), r_Arm_Bottom(:, 2)];
z_Array = [r_Shoulder(:, 3), l_Shoulder(:, 3), l_Hip(:, 3), r_Hip(:, 3), r_Shoulder(:, 3), ...
    nan_Array, zero_Array + l_P_Fixed(3), l_Arm_Bottom(:, 3), nan_Array, zero_Array + r_P_Fixed(3), r_Arm_Bottom(:, 3)];

plot3(x_Array, y_Array, z_Array)
hold on
scatter3(r_Shoulder(1), r_Shoulder(2), r_Shoulder(3), 'r', 'filled')
scatter3(r_Arm_Bottom(1), r_Arm_Bottom(2), r_Arm_Bottom(3), 'r', 'filled')
scatter3(l_Shoulder(1), l_Shoulder(2), l_Shoulder(3), 'b', 'filled')
scatter3(l_Arm_Bottom(1), l_Arm_Bottom(2), l_Arm_Bottom(3), 'b', 'filled')
hold off
daspect([1,1,1])

% anime = AnimeAndData(time, x_Array, y_Array, z_Array);
% plot_Lim = 4 * [-1, 1];
% xlim(anime.axAnime, plot_Lim)
% ylim(anime.axAnime, plot_Lim)
% zlim(anime.axAnime, plot_Lim)
% view(anime.axAnime, [-1,-1,-1])

















































