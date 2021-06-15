

l_Alpha_Hand = deg2rad(-90);
l_Beta_Hand = deg2rad(0);

m_Hand = 1;
length_Hand = 1;
g = 1;

l_F_X = 0;
l_F_Y = 0;
l_F_Z = 0;

l_X_Fixed = 0;
l_Y_Fixed = 0;
l_Z_Fixed = 0;

l_Tau_Alpha_Shoulder = 0.3;
l_Tau_Beta_Shoulder = 0.;


time = 0:1e-2:100;
q = [l_Alpha_Hand, 0, l_Beta_Hand, 0]';

[time, q] = ode45(@(t,q) ddt_L_Arm(t, q, g, length_Hand, m_Hand, l_Tau_Beta_Shoulder, l_Tau_Alpha_Shoulder, l_F_X, l_F_Y, l_F_Z), time, q);

l_Alpha_Hand = q(:, 1);
dl_Alpha_Hand = q(:, 2);
l_Beta_Hand = q(:, 3);
dl_Beta_Hand = q(:, 4);

l_Arm_Bottom = FRD_L_Arm_Bottom(l_Alpha_Hand,l_Beta_Hand,l_X_Fixed,l_Y_Fixed,l_Z_Fixed,length_Hand);

nan_Array = nan(size(time, 1), 1);
zero_Array = zeros(size(time, 1), 1);

x_Array = [zero_Array, l_Arm_Bottom(:,1)];
y_Array = [zero_Array, l_Arm_Bottom(:,2)];
z_Array = [zero_Array, l_Arm_Bottom(:,3)];


anime = AnimeAndData(time, x_Array, y_Array, z_Array);
plot_Lim = 2 * [-1, 1];
xlim(anime.axAnime, plot_Lim)
ylim(anime.axAnime, plot_Lim)
zlim(anime.axAnime, plot_Lim)
view(anime.axAnime, [1, 0, 0])

dockfig(1)
plot(time, [l_Alpha_Hand, l_Beta_Hand])

dockfig(2)
plot(time, l_Arm_Bottom)




