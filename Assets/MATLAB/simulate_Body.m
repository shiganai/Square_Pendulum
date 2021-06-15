
clear all

width_Body = 1;
height_Body = 2;
depth_Body = 0.5;
m_Body = 10;
m_Hand = 1;
length_Hand = 1;
g = 0.;

alpha_Body = deg2rad(0);
beta_Body = deg2rad(0);
gamma_Body = deg2rad(0);
x_Head = 0;
y_Head = 0;
z_Head = 0;

r_Alpha_Hand = deg2rad(-0);
r_Beta_Hand = deg2rad(0);

l_Alpha_Hand = deg2rad(-0);
l_Beta_Hand = deg2rad(0);

r_Tau_Alpha_Shoulder = 0;
l_Tau_Alpha_Shoulder = -0.;

r_Tau_Beta_Shoulder = -1;
l_Tau_Beta_Shoulder = -0.;

r_F_X = 0;
r_F_Y = 0;
r_F_Z = 0;

l_F_X = 0;
l_F_Y = 0;
l_F_Z = 0;


time = 0:1e-2:10;
q = [alpha_Body, 1, beta_Body, 0, gamma_Body, 0, x_Head, 0, y_Head, 0, z_Head, -1]';

tic
[time, q] = ode45(@(t,q) ...
    ddt_Body(t, q, g, m_Body, depth_Body, height_Body, width_Body, r_Alpha_Hand, l_Alpha_Hand, r_Tau_Beta_Shoulder, r_Tau_Alpha_Shoulder, l_Tau_Beta_Shoulder, l_Tau_Alpha_Shoulder, r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z)...
    , time, q);
toc

alpha_Body = q(:, 1);
dalpha_Body = q(:, 2);
beta_Body = q(:, 3);
dbeta_Body = q(:, 4);
gamma_Body = q(:, 5);
dgamma_Body = q(:, 6);
x_Head = q(:, 7);
dx_Head = q(:, 8);
y_Head = q(:, 9);
dy_Head = q(:, 10);
z_Head = q(:, 11);
dz_Head = q(:, 12);

r_Shoulder = FRD_R_Shoulder(alpha_Body,beta_Body,gamma_Body,width_Body,x_Head,y_Head,z_Head);
r_Hip = FRD_R_Hip(alpha_Body,beta_Body,gamma_Body,height_Body,width_Body,x_Head,y_Head,z_Head);
l_Shoulder = FRD_L_Shoulder(alpha_Body,beta_Body,gamma_Body,width_Body,x_Head,y_Head,z_Head);
l_Hip = FRD_L_Hip(alpha_Body,beta_Body,gamma_Body,height_Body,width_Body,x_Head,y_Head,z_Head);


x_Array = [r_Shoulder(:, 1), l_Shoulder(:, 1), l_Hip(:, 1), r_Hip(:, 1), r_Shoulder(:, 1), ...
    ];
y_Array = [r_Shoulder(:, 2), l_Shoulder(:, 2), l_Hip(:, 2), r_Hip(:, 2), r_Shoulder(:, 2), ...
    ];
z_Array = [r_Shoulder(:, 3), l_Shoulder(:, 3), l_Hip(:, 3), r_Hip(:, 3), r_Shoulder(:, 3), ...
    ];


anime = AnimeAndData(time, x_Array, y_Array, z_Array);
plot_Lim = 4 * [-1, 1];
xlim(anime.axAnime, plot_Lim)
ylim(anime.axAnime, plot_Lim)
zlim(anime.axAnime, plot_Lim)
view(anime.axAnime, [1,1,1])








































