
clear all
tic

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

%%

time = 0:1e-2:50;
q = [r_Alpha_Hand, 0, r_Beta_Hand, 0, l_Alpha_Hand, 0, l_Beta_Hand, 0, ...
    alpha_Body, 0, beta_Body, 0, gamma_Body, 0, ...
    x_Head, 0, y_Head, 0, z_Head, 0]';

r_Tau_Alpha_Shoulder = -0.;
l_Tau_Alpha_Shoulder = -0.;

r_Tau_Beta_Shoulder = -0.;
l_Tau_Beta_Shoulder = -0.;

toc
tic
[time_1, q_1] = ode45(@(t,q) ddt_FFD(t,q,r_P_Fixed,l_P_Fixed, g, length_Hand, m_Hand, m_Body, width_Body, height_Body, depth_Body, l_Tau_Alpha_Shoulder, r_Tau_Alpha_Shoulder, l_Tau_Beta_Shoulder, r_Tau_Beta_Shoulder), time, q);
toc

r_Tau_Alpha_Shoulder = -0.;
l_Tau_Alpha_Shoulder = -0.;

r_Tau_Beta_Shoulder = -0.;
l_Tau_Beta_Shoulder = -0.;

time = time + time_1(end);

tic
[time_2, q_2] = ode45(@(t,q) ddt_FFD(t,q,r_P_Fixed,l_P_Fixed, g, length_Hand, m_Hand, m_Body, width_Body, height_Body, depth_Body, l_Tau_Alpha_Shoulder, r_Tau_Alpha_Shoulder, l_Tau_Beta_Shoulder, r_Tau_Beta_Shoulder), time, q_1(end,:));
toc

time = [time_1(1:end-1); time_2];
q = [q_1(1:end-1,:); q_2];

r_Alpha_Hand = q(:, 1);
dr_Alpha_Hand = q(:, 2);

r_Beta_Hand = q(:, 3);
dr_Beta_Hand = q(:, 4);

l_Alpha_Hand = q(:, 5);
dl_Alpha_Hand = q(:, 6);

l_Beta_Hand = q(:, 7);
dl_Beta_Hand = q(:, 8);

alpha_Body = q(:, 9);
dalpha_Body = q(:, 10);
ddalpha_Body = fnval(fnder(spline(time, dalpha_Body)), time);

beta_Body = q(:, 11);
dbeta_Body = q(:, 12);
ddbeta_Body = fnval(fnder(spline(time, dbeta_Body)), time);

gamma_Body = q(:, 13);
dgamma_Body = q(:, 14);
ddgamma_Body = fnval(fnder(spline(time, dgamma_Body)), time);

x_Head = q(:, 15);
dx_Head = q(:, 16);

y_Head = q(:, 17);
dy_Head = q(:, 18);

z_Head = q(:, 19);
dz_Head = q(:, 20);

%%
r_Arm_Bottom = FRD_R_Arm_Bottom(length_Hand,r_Alpha_Hand,r_Beta_Hand,r_X_Fixed,r_Y_Fixed,r_Z_Fixed);
l_Arm_Bottom = FRD_L_Arm_Bottom(l_Alpha_Hand,l_Beta_Hand,l_X_Fixed,l_Y_Fixed,l_Z_Fixed,length_Hand);

r_Shoulder = FRD_R_Shoulder(alpha_Body,beta_Body,gamma_Body,width_Body,x_Head,y_Head,z_Head);
l_Shoulder = FRD_L_Shoulder(alpha_Body,beta_Body,gamma_Body,width_Body,x_Head,y_Head,z_Head);

r_Hip = FRD_R_Hip(alpha_Body,beta_Body,gamma_Body,height_Body,width_Body,x_Head,y_Head,z_Head);
l_Hip = FRD_L_Hip(alpha_Body,beta_Body,gamma_Body,height_Body,width_Body,x_Head,y_Head,z_Head);

nan_Array = nan(size(l_Shoulder, 1), 1);
zero_Array = zeros(size(l_Shoulder, 1), 1);

x_Array = [r_Shoulder(:, 1), l_Shoulder(:, 1), l_Hip(:, 1), r_Hip(:, 1), r_Shoulder(:, 1), ...
    nan_Array, zero_Array + l_P_Fixed(1), l_Arm_Bottom(:, 1), nan_Array, zero_Array + r_P_Fixed(1), r_Arm_Bottom(:, 1)];
y_Array = [r_Shoulder(:, 2), l_Shoulder(:, 2), l_Hip(:, 2), r_Hip(:, 2), r_Shoulder(:, 2), ...
    nan_Array, zero_Array + l_P_Fixed(2), l_Arm_Bottom(:, 2), nan_Array, zero_Array + r_P_Fixed(2), r_Arm_Bottom(:, 2)];
z_Array = [r_Shoulder(:, 3), l_Shoulder(:, 3), l_Hip(:, 3), r_Hip(:, 3), r_Shoulder(:, 3), ...
    nan_Array, zero_Array + l_P_Fixed(3), l_Arm_Bottom(:, 3), nan_Array, zero_Array + r_P_Fixed(3), r_Arm_Bottom(:, 3)];

% x_Array = [r_Shoulder(:, 1), l_Shoulder(:, 1), l_Hip(:, 1), r_Hip(:, 1), r_Shoulder(:, 1), ...
%     nan_Array, zero_Array + r_P_Fixed(1), r_Arm_Bottom(:, 1)];
% y_Array = [r_Shoulder(:, 2), l_Shoulder(:, 2), l_Hip(:, 2), r_Hip(:, 2), r_Shoulder(:, 2), ...
%     nan_Array, zero_Array + r_P_Fixed(2), r_Arm_Bottom(:, 2)];
% z_Array = [r_Shoulder(:, 3), l_Shoulder(:, 3), l_Hip(:, 3), r_Hip(:, 3), r_Shoulder(:, 3), ...
%     nan_Array, zero_Array + r_P_Fixed(3), r_Arm_Bottom(:, 3)];

anime = AnimeAndData(time, x_Array, y_Array, z_Array);
plot_Lim = 4 * [-1, 1];
xlim(anime.axAnime, plot_Lim)
ylim(anime.axAnime, plot_Lim)
zlim(anime.axAnime, plot_Lim)
view(anime.axAnime, [1,-0,-0])

dockfig(1)
plot(time, [alpha_Body, beta_Body, gamma_Body])

dockfig(2)
plot(time, [dbeta_Body, dgamma_Body])

dockfig(3)
plot(time, [ddbeta_Body, ddgamma_Body])










































