
clear all
tic

parallel.defaultClusterProfile('local');
c = parcluster();

syms t real

%% Start about r_Arm

syms r_Alpha_Hand_Pre(t)
syms r_Beta_Hand_Pre(t)
syms r_Gamma_Hand_Pre(t)
syms r_Alpha_Hand dr_Alpha_Hand ddr_Alpha_Hand real
syms r_Beta_Hand dr_Beta_Hand ddr_Beta_Hand real
syms r_Gamma_Hand dr_Gamma_Hand ddr_Gamma_Hand real
syms r_Tau_Alpha_Shoulder real
syms r_Tau_Beta_Shoulder real
syms r_Tau_Gamma_Shoulder real

syms_Replaced = [
    r_Alpha_Hand_Pre diff(r_Alpha_Hand_Pre, t) diff(r_Alpha_Hand_Pre, t, t), ...
    r_Beta_Hand_Pre diff(r_Beta_Hand_Pre, t) diff(r_Beta_Hand_Pre, t, t), ...
    r_Gamma_Hand_Pre diff(r_Gamma_Hand_Pre, t) diff(r_Gamma_Hand_Pre, t, t), ...
    ];

syms_Replacing = [
    r_Alpha_Hand dr_Alpha_Hand ddr_Alpha_Hand ...
    r_Beta_Hand dr_Beta_Hand ddr_Beta_Hand ...
    r_Gamma_Hand dr_Gamma_Hand ddr_Gamma_Hand ...
    ];

%% Rotate
r_Rotate_Matrix = ...
    [cos(r_Gamma_Hand_Pre), 0, sin(r_Gamma_Hand_Pre);0, 1, 0;-sin(r_Gamma_Hand_Pre), 0, cos(r_Gamma_Hand_Pre);]' ...
    * ...
    [cos(r_Beta_Hand_Pre), -sin(r_Beta_Hand_Pre), 0; sin(r_Beta_Hand_Pre), cos(r_Beta_Hand_Pre), 0; 0, 0, 1;]' ...
    * ...
    [1, 0, 0; 0, cos(r_Alpha_Hand_Pre), -sin(r_Alpha_Hand_Pre); 0, sin(r_Alpha_Hand_Pre), cos(r_Alpha_Hand_Pre);]' ...
    * ...
    1;

dr_Rotate_Matrix = diff(r_Rotate_Matrix, t);

%% 
ohm_R = formula(r_Rotate_Matrix' * dr_Rotate_Matrix);

omega_X = ohm_R(2,3);
omega_Y = ohm_R(3,1);
omega_Z = ohm_R(1,2);

omega_R = simplify([omega_X; omega_Y; omega_Z]);

omega_Tmp = simplify(subs(omega_R, syms_Replaced, syms_Replacing));
unit_Vector_R = coeffs_Vector(omega_Tmp, [dr_Alpha_Hand, dr_Beta_Hand, dr_Gamma_Hand]);

r_Tau_Vec = r_Tau_Alpha_Shoulder * unit_Vector_R(:,1) ...
    + r_Tau_Beta_Shoulder * unit_Vector_R(:,2) ...
    + r_Tau_Gamma_Shoulder * unit_Vector_R(:,3);

%% Start about l_Arm

syms l_Alpha_Hand_Pre(t)
syms l_Beta_Hand_Pre(t)
syms l_Gamma_Hand_Pre(t)
syms l_Alpha_Hand dl_Alpha_Hand ddl_Alpha_Hand real
syms l_Beta_Hand dl_Beta_Hand ddl_Beta_Hand real
syms l_Gamma_Hand dl_Gamma_Hand ddl_Gamma_Hand real
syms l_Tau_Alpha_Shoulder real
syms l_Tau_Beta_Shoulder real
syms l_Tau_Gamma_Shoulder real

syms_Replaced = [
    l_Alpha_Hand_Pre diff(l_Alpha_Hand_Pre, t) diff(l_Alpha_Hand_Pre, t, t), ...
    l_Beta_Hand_Pre diff(l_Beta_Hand_Pre, t) diff(l_Beta_Hand_Pre, t, t), ...
    l_Gamma_Hand_Pre diff(l_Gamma_Hand_Pre, t) diff(l_Gamma_Hand_Pre, t, t), ...
    ];

syms_Replacing = [
    l_Alpha_Hand dl_Alpha_Hand ddl_Alpha_Hand ...
    l_Beta_Hand dl_Beta_Hand ddl_Beta_Hand ...
    l_Gamma_Hand dl_Gamma_Hand ddl_Gamma_Hand ...
    ];

%% Rotate
l_Rotate_Matrix = ...
    [cos(-l_Gamma_Hand_Pre), 0, sin(-l_Gamma_Hand_Pre);0, 1, 0;-sin(-l_Gamma_Hand_Pre), 0, cos(-l_Gamma_Hand_Pre);]' ...
    * ...
    [cos(-l_Beta_Hand_Pre), -sin(-l_Beta_Hand_Pre), 0; sin(-l_Beta_Hand_Pre), cos(-l_Beta_Hand_Pre), 0; 0, 0, 1;]' ...
    * ...
    [1, 0, 0; 0, cos(l_Alpha_Hand_Pre), -sin(l_Alpha_Hand_Pre); 0, sin(l_Alpha_Hand_Pre), cos(l_Alpha_Hand_Pre);]' ...
    * ...
    1;

dl_Rotate_Matrix = diff(l_Rotate_Matrix, t);

%% 
ohm = formula(l_Rotate_Matrix' * dl_Rotate_Matrix);

omega_X = ohm(2,3);
omega_Y = ohm(3,1);
omega_Z = ohm(1,2);

omega_L = simplify([omega_X; omega_Y; omega_Z]);

omega_Tmp = simplify(subs(omega_L, syms_Replaced, syms_Replacing));
unit_Vector_L = coeffs_Vector(omega_Tmp, [dl_Alpha_Hand, dl_Beta_Hand, dl_Gamma_Hand]);

l_Tau_Vec = l_Tau_Alpha_Shoulder * unit_Vector_L(:,1) ...
    + l_Tau_Beta_Shoulder * unit_Vector_L(:,2) ...
    + l_Tau_Gamma_Shoulder * unit_Vector_L(:,3);

%% find external_Tau vector
external_Tau_Vec = r_Tau_Vec + l_Tau_Vec;

%% Start about body
syms width_Body height_Body depth_Body real
syms m_Body real
syms g real

syms r_F_X r_F_Y r_F_Z real
syms l_F_X l_F_Y l_F_Z real

syms alpha_Body_Pre(t)
syms beta_Body_Pre(t)
syms gamma_Body_Pre(t)
syms x_Head_Pre(t) y_Head_Pre(t) z_Head_Pre(t)

%%

syms alpha_Body dalpha_Body ddalpha_Body real
syms beta_Body dbeta_Body ddbeta_Body real
syms gamma_Body dgamma_Body ddgamma_Body real
syms x_Head dx_Head ddx_Head real
syms y_Head dy_Head ddy_Head real
syms z_Head dz_Head ddz_Head real

syms_Replaced = [
    alpha_Body_Pre diff(alpha_Body_Pre, t) diff(alpha_Body_Pre, t, t), ...
    beta_Body_Pre diff(beta_Body_Pre, t) diff(beta_Body_Pre, t, t), ...
    gamma_Body_Pre diff(gamma_Body_Pre, t) diff(gamma_Body_Pre, t, t), ...
    x_Head_Pre diff(x_Head_Pre, t) diff(x_Head_Pre, t, t), ...
    y_Head_Pre diff(y_Head_Pre, t) diff(y_Head_Pre, t, t), ...
    z_Head_Pre diff(z_Head_Pre, t) diff(z_Head_Pre, t, t), ...
    ];

syms_Replacing = [
    alpha_Body dalpha_Body ddalpha_Body ...
    beta_Body dbeta_Body ddbeta_Body ...
    gamma_Body dgamma_Body ddgamma_Body ...
    x_Head dx_Head ddx_Head ...
    y_Head dy_Head ddy_Head ...
    z_Head dz_Head ddz_Head ...
    ];

%%
I_Body = 1/12 * m_Body * [
    height_Body^2 + depth_Body^2, 0, 0;
    0, width_Body^2 + depth_Body^2, 0;
    0, 0, width_Body^2 + height_Body^2;
    ];

%%
head = [0, 0, 0];
r_Shoulder = [width_Body/2, 0, 0];
l_Shoulder = [-width_Body/2, 0, 0];
r_Hip = [width_Body/2, height_Body, 0];
l_Hip = [-width_Body/2, height_Body, 0];
g_Body = (r_Shoulder + l_Shoulder + r_Hip + l_Hip)/4;

%{
%% rotate beta_Body around y
tauvec_Beta = symfun([0,1,0,1], t);
trans_Matrix_Beta = [cos(beta_Body_Pre), 0, sin(beta_Body_Pre), 0; 0, 1, 0, 0; -sin(beta_Body_Pre), 0, cos(beta_Body_Pre), 0; 0, 0, 0, 1]';

head = head * trans_Matrix_Beta;
r_Shoulder = r_Shoulder * trans_Matrix_Beta;
l_Shoulder = l_Shoulder * trans_Matrix_Beta;
r_Hip = r_Hip * trans_Matrix_Beta;
l_Hip = l_Hip * trans_Matrix_Beta;
g_Body = g_Body * trans_Matrix_Beta;

%% rotate gamma_Body around z
tauvec_Gamma = symfun([0,0,1,1], t);
trans_Matrix_Gamma = [cos(gamma_Body_Pre), -sin(gamma_Body_Pre), 0, 0; sin(gamma_Body_Pre), cos(gamma_Body_Pre), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1]';

head = head * trans_Matrix_Gamma;
r_Shoulder = r_Shoulder * trans_Matrix_Gamma;
l_Shoulder = l_Shoulder * trans_Matrix_Gamma;
r_Hip = r_Hip * trans_Matrix_Gamma;
l_Hip = l_Hip * trans_Matrix_Gamma;
g_Body = g_Body * trans_Matrix_Gamma;

tauvec_Beta = tauvec_Beta * trans_Matrix_Gamma;

%% rotate beta_Body around x
tauvec_Alpha = symfun([1,0,0,1], t);
trans_Matrix_Alpha = [1, 0, 0, 0; 0, cos(alpha_Body_Pre), -sin(alpha_Body_Pre), 0; 0, sin(alpha_Body_Pre), cos(alpha_Body_Pre), 0; 0, 0, 0, 1]';

head = head * trans_Matrix_Alpha;
r_Shoulder = r_Shoulder * trans_Matrix_Alpha;
l_Shoulder = l_Shoulder * trans_Matrix_Alpha;
r_Hip = r_Hip * trans_Matrix_Alpha;
l_Hip = l_Hip * trans_Matrix_Alpha;
g_Body = g_Body * trans_Matrix_Alpha;

tauvec_Beta = tauvec_Beta * trans_Matrix_Alpha;
tauvec_Gamma = tauvec_Gamma * trans_Matrix_Alpha;

%% move origin
trans_Matrix_Origin = [1, 0, 0, x_Head_Pre; 0, 1, 0, y_Head_Pre; 0, 0, 1, z_Head_Pre; 0, 0, 0, 1]';

head = head * trans_Matrix_Origin;
r_Shoulder = r_Shoulder * trans_Matrix_Origin;
l_Shoulder = l_Shoulder * trans_Matrix_Origin;
r_Hip = r_Hip * trans_Matrix_Origin;
l_Hip = l_Hip * trans_Matrix_Origin;
g_Body = g_Body * trans_Matrix_Origin;
%}

%{/
%% Rotate
rotate_Matrix_Body = ...
    [cos(beta_Body_Pre), 0, sin(beta_Body_Pre);0, 1, 0;-sin(beta_Body_Pre), 0, cos(beta_Body_Pre);]' ...
    * ...
    [cos(gamma_Body_Pre), -sin(gamma_Body_Pre), 0;sin(gamma_Body_Pre), cos(gamma_Body_Pre), 0;0, 0, 1;]' ...
    * ...
    [1, 0, 0;0, cos(alpha_Body_Pre), -sin(alpha_Body_Pre);0, sin(alpha_Body_Pre), cos(alpha_Body_Pre);]' ...
    * ...
    1;

drotate_Matrix_Body = diff(rotate_Matrix_Body, t);

head = formula(head * rotate_Matrix_Body);
r_Shoulder = formula(r_Shoulder * rotate_Matrix_Body);
l_Shoulder = formula(l_Shoulder * rotate_Matrix_Body);
r_Hip = formula(r_Hip * rotate_Matrix_Body);
l_Hip = formula(l_Hip * rotate_Matrix_Body);
g_Body = formula(g_Body * rotate_Matrix_Body);

%% Move
head = head + [x_Head_Pre, y_Head_Pre, z_Head_Pre];
r_Shoulder = r_Shoulder + [x_Head_Pre, y_Head_Pre, z_Head_Pre];
l_Shoulder = l_Shoulder + [x_Head_Pre, y_Head_Pre, z_Head_Pre];
r_Hip = r_Hip + [x_Head_Pre, y_Head_Pre, z_Head_Pre];
l_Hip = l_Hip + [x_Head_Pre, y_Head_Pre, z_Head_Pre];
g_Body = g_Body + [x_Head_Pre, y_Head_Pre, z_Head_Pre];

%% 
ohm = formula(rotate_Matrix_Body' * drotate_Matrix_Body);

omega_X = ohm(2,3);
omega_Y = ohm(3,1);
omega_Z = ohm(1,2);

omega_Body = simplify([omega_X; omega_Y; omega_Z]);
omega_Euler_Body = simplify(rotate_Matrix_Body * omega_Body);

omega_Tmp = simplify(subs(omega_Body, syms_Replaced, syms_Replacing));
unit_Vector_Body = coeffs_Vector(omega_Tmp, [dalpha_Body, dbeta_Body, dgamma_Body]);
unit_Vector_Body = subs(unit_Vector_Body, syms_Replacing, syms_Replaced);
%}

%%
head = formula(head);
r_Shoulder = formula(r_Shoulder);
l_Shoulder = formula(l_Shoulder);
r_Hip = formula(r_Hip);
l_Hip = formula(l_Hip);
g_Body = formula(g_Body);
% tauvec_Beta = formula(tauvec_Beta);
% tauvec_Gamma = formula(tauvec_Gamma);
% tauvec_Alpha = formula(tauvec_Alpha);

v_G_Body = diff(g_Body, t);

T = ...
    1/2 * m_Body * (v_G_Body * v_G_Body')...
    + ...
    1/2 * (omega_Euler_Body' * (I_Body * omega_Euler_Body))...
    + ...
    0;

U = ...
    m_Body * g * g_Body(3)...
    +...
    0;

L = T - U;

% coeffs_Tau_Body = inv(unit_Vector_Body(:,1:3)) * external_Tau_Vec;
coeffs_Tau_Body = unit_Vector_Body(:,1:3)\external_Tau_Vec;

coeffs_Tau_Alpha_Body = coeffs_Tau_Body(1);
coeffs_Tau_Beta_Body = coeffs_Tau_Body(2);
coeffs_Tau_Gamma_Body = coeffs_Tau_Body(3);

%%
equations = [
    -functionalDerivative(L, alpha_Body_Pre) == coeffs_Tau_Alpha_Body + ([l_F_X, l_F_Y, l_F_Z] * diff(l_Shoulder, alpha_Body_Pre)') + ([r_F_X, r_F_Y, r_F_Z] * diff(r_Shoulder, alpha_Body_Pre)');
    -functionalDerivative(L, beta_Body_Pre) == coeffs_Tau_Beta_Body + ([l_F_X, l_F_Y, l_F_Z] * diff(l_Shoulder, beta_Body_Pre)') + ([r_F_X, r_F_Y, r_F_Z] * diff(r_Shoulder, beta_Body_Pre)');
    -functionalDerivative(L, gamma_Body_Pre) == coeffs_Tau_Gamma_Body + ([l_F_X, l_F_Y, l_F_Z] * diff(l_Shoulder, gamma_Body_Pre)') + ([r_F_X, r_F_Y, r_F_Z] * diff(r_Shoulder, gamma_Body_Pre)');
    -functionalDerivative(L, x_Head_Pre) == 0 + ([l_F_X, l_F_Y, l_F_Z] * diff(l_Shoulder, x_Head_Pre)') + ([r_F_X, r_F_Y, r_F_Z] * diff(r_Shoulder, x_Head_Pre)');
    -functionalDerivative(L, y_Head_Pre) == 0 + ([l_F_X, l_F_Y, l_F_Z] * diff(l_Shoulder, y_Head_Pre)') + ([r_F_X, r_F_Y, r_F_Z] * diff(r_Shoulder, y_Head_Pre)');
    -functionalDerivative(L, z_Head_Pre) == 0 + ([l_F_X, l_F_Y, l_F_Z] * diff(l_Shoulder, z_Head_Pre)') + ([r_F_X, r_F_Y, r_F_Z] * diff(r_Shoulder, z_Head_Pre)');
    ];

equations = subs(equations, syms_Replaced, syms_Replacing);

%% Full forward dynamics
%{
variables = [ddalpha_Body, ddbeta_Body, ddgamma_Body, ddx_Head, ddy_Head, ddz_Head];

[A, B] = equationsToMatrix(equations, variables);
toc
tic
X = inv(A)*B;
toc

job = createJob(c);
createTask(job, @matlabFunction, 1,{X(1), X(2), X(3), X(4), X(5), X(6), ...
    'file', 'FFD_Dds_Body.m', 'outputs', ...
    {'ddalpha_Body', 'ddbeta_Body', 'ddgamma_Body', 'ddx_Head', 'ddy_Head', 'ddz_Head'}});
submit(job)
job.Tasks

%{
ddr_Shoulder = formula(diff(r_Shoulder, t, t))';
ddr_Shoulder = subs(ddr_Shoulder, syms_Replaced, syms_Replacing);
ddr_Shoulder = subs(ddr_Shoulder, variables, X');

coeffs_Ddr_Shoulder_FFD = coeffs_Vector(ddr_Shoulder, [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z]);

size(coeffs_Ddr_Shoulder_FFD)

% Make whole of matrix
%{
job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Shoulder_FFD(1,1), coeffs_Ddr_Shoulder_FFD(1,2), coeffs_Ddr_Shoulder_FFD(1,3), coeffs_Ddr_Shoulder_FFD(1,4), coeffs_Ddr_Shoulder_FFD(1,5), coeffs_Ddr_Shoulder_FFD(1,6), coeffs_Ddr_Shoulder_FFD(1,7), ...
    coeffs_Ddr_Shoulder_FFD(2,1), coeffs_Ddr_Shoulder_FFD(2,2), coeffs_Ddr_Shoulder_FFD(2,3), coeffs_Ddr_Shoulder_FFD(2,4), coeffs_Ddr_Shoulder_FFD(2,5), coeffs_Ddr_Shoulder_FFD(2,6), coeffs_Ddr_Shoulder_FFD(2,7), ...
    coeffs_Ddr_Shoulder_FFD(3,1), coeffs_Ddr_Shoulder_FFD(3,2), coeffs_Ddr_Shoulder_FFD(3,3), coeffs_Ddr_Shoulder_FFD(3,4), coeffs_Ddr_Shoulder_FFD(3,5), coeffs_Ddr_Shoulder_FFD(3,6), coeffs_Ddr_Shoulder_FFD(3,7), ...
    'file', 'FFD_Coeffs_Ddr_Shoulder.m', 'outputs', ...
    {...
    'A11','A12','A13','A14','A15','A16','A17',...
    'A21','A22','A23','A24','A25','A26','A27',...
    'A31','A32','A33','A34','A35','A36','A37',...
    }});
submit(job)
job.Tasks
%}

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Shoulder_FFD(1,1), coeffs_Ddr_Shoulder_FFD(1,2), coeffs_Ddr_Shoulder_FFD(1,3), coeffs_Ddr_Shoulder_FFD(1,4), coeffs_Ddr_Shoulder_FFD(1,5), coeffs_Ddr_Shoulder_FFD(1,6), coeffs_Ddr_Shoulder_FFD(1,7), ...
    'file', 'FFD_Coeffs_Ddr_Shoulder_Row1.m', 'outputs', ...
    {...
    'A11','A12','A13','A14','A15','A16','A17',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Shoulder_FFD(2,1), coeffs_Ddr_Shoulder_FFD(2,2), coeffs_Ddr_Shoulder_FFD(2,3), coeffs_Ddr_Shoulder_FFD(2,4), coeffs_Ddr_Shoulder_FFD(2,5), coeffs_Ddr_Shoulder_FFD(2,6), coeffs_Ddr_Shoulder_FFD(2,7), ...
    'file', 'FFD_Coeffs_Ddr_Shoulder_Row2.m', 'outputs', ...
    {...
    'A21','A22','A23','A24','A25','A26','A27',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Shoulder_FFD(3,1), coeffs_Ddr_Shoulder_FFD(3,2), coeffs_Ddr_Shoulder_FFD(3,3), coeffs_Ddr_Shoulder_FFD(3,4), coeffs_Ddr_Shoulder_FFD(3,5), coeffs_Ddr_Shoulder_FFD(3,6), coeffs_Ddr_Shoulder_FFD(3,7), ...
    'file', 'FFD_Coeffs_Ddr_Shoulder_Row3.m', 'outputs', ...
    {...
    'A31','A32','A33','A34','A35','A36','A37',...
    }});
submit(job)
job.Tasks

ddl_Shoulder = formula(diff(l_Shoulder, t, t))';
ddl_Shoulder = subs(ddl_Shoulder, syms_Replaced, syms_Replacing);
ddl_Shoulder = subs(ddl_Shoulder, variables, X');

coeffs_Ddl_Shoulder_FFD = coeffs_Vector(ddl_Shoulder, [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z]);

size(coeffs_Ddl_Shoulder_FFD)

% Make whole of matrix
%{
job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddl_Shoulder_FFD(1,1), coeffs_Ddl_Shoulder_FFD(1,2), coeffs_Ddl_Shoulder_FFD(1,3), coeffs_Ddl_Shoulder_FFD(1,4), coeffs_Ddl_Shoulder_FFD(1,5), coeffs_Ddl_Shoulder_FFD(1,6), coeffs_Ddl_Shoulder_FFD(1,7), ...
    coeffs_Ddl_Shoulder_FFD(2,1), coeffs_Ddl_Shoulder_FFD(2,2), coeffs_Ddl_Shoulder_FFD(2,3), coeffs_Ddl_Shoulder_FFD(2,4), coeffs_Ddl_Shoulder_FFD(2,5), coeffs_Ddl_Shoulder_FFD(2,6), coeffs_Ddl_Shoulder_FFD(2,7), ...
    coeffs_Ddl_Shoulder_FFD(3,1), coeffs_Ddl_Shoulder_FFD(3,2), coeffs_Ddl_Shoulder_FFD(3,3), coeffs_Ddl_Shoulder_FFD(3,4), coeffs_Ddl_Shoulder_FFD(3,5), coeffs_Ddl_Shoulder_FFD(3,6), coeffs_Ddl_Shoulder_FFD(3,7), ...
    'file', 'FFD_Coeffs_Ddl_Shoulder.m', 'outputs', ...
    {...
    'A11','A12','A13','A14','A15','A16','A17',...
    'A21','A22','A23','A24','A25','A26','A27',...
    'A31','A32','A33','A34','A35','A36','A37',...
    }});
submit(job)
job.Tasks
%}

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddl_Shoulder_FFD(1,1), coeffs_Ddl_Shoulder_FFD(1,2), coeffs_Ddl_Shoulder_FFD(1,3), coeffs_Ddl_Shoulder_FFD(1,4), coeffs_Ddl_Shoulder_FFD(1,5), coeffs_Ddl_Shoulder_FFD(1,6), coeffs_Ddl_Shoulder_FFD(1,7), ...
    'file', 'FFD_Coeffs_Ddl_Shoulder_Row1.m', 'outputs', ...
    {...
    'A11','A12','A13','A14','A15','A16','A17',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddl_Shoulder_FFD(2,1), coeffs_Ddl_Shoulder_FFD(2,2), coeffs_Ddl_Shoulder_FFD(2,3), coeffs_Ddl_Shoulder_FFD(2,4), coeffs_Ddl_Shoulder_FFD(2,5), coeffs_Ddl_Shoulder_FFD(2,6), coeffs_Ddl_Shoulder_FFD(2,7), ...
    'file', 'FFD_Coeffs_Ddl_Shoulder_Row2.m', 'outputs', ...
    {...
    'A21','A22','A23','A24','A25','A26','A27',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddl_Shoulder_FFD(3,1), coeffs_Ddl_Shoulder_FFD(3,2), coeffs_Ddl_Shoulder_FFD(3,3), coeffs_Ddl_Shoulder_FFD(3,4), coeffs_Ddl_Shoulder_FFD(3,5), coeffs_Ddl_Shoulder_FFD(3,6), coeffs_Ddl_Shoulder_FFD(3,7), ...
    'file', 'FFD_Coeffs_Ddl_Shoulder_Row3.m', 'outputs', ...
    {...
    'A31','A32','A33','A34','A35','A36','A37',...
    }});
submit(job)
job.Tasks
%}
%}

%% Half forward dynamics
%{/
equations = subs(equations, [l_Tau_Beta_Shoulder, l_Tau_Gamma_Shoulder, ], [r_Tau_Beta_Shoulder, r_Tau_Gamma_Shoulder, ]);
variables = [ddalpha_Body, ddx_Head, ddy_Head, ddz_Head, r_Tau_Beta_Shoulder, r_Tau_Gamma_Shoulder];
% variables = [ddalpha_Body, ddbeta_Body, ddgamma_Body, ddx_Head, ddy_Head, ddz_Head];

[A, B] = equationsToMatrix(equations, variables);
toc
simplify(det(A))
tic
X = inv(A)*B;
toc

job = createJob(c);
createTask(job, @matlabFunction, 1,{X(1), X(2), X(3), X(4), X(5), X(6), ...
    'file', 'HFD_Dds_Body.m', 'outputs', ...
    {'ddalpha_Body', 'ddx_Head', 'ddy_Head', 'ddz_Head', 'r_Tau_Beta_Shoulder', 'r_Tau_Gamma_Shoulder'}});
submit(job)
job.Tasks

%{
coeffs_R_Tau_Alpba_Shoulder_HFD = coeffs_Vector(X(5), [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z]);

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_R_Tau_Alpba_Shoulder_HFD(1,1), coeffs_R_Tau_Alpba_Shoulder_HFD(1,2), coeffs_R_Tau_Alpba_Shoulder_HFD(1,3), ...
    coeffs_R_Tau_Alpba_Shoulder_HFD(1,4), coeffs_R_Tau_Alpba_Shoulder_HFD(1,5), coeffs_R_Tau_Alpba_Shoulder_HFD(1,6), ...
    'file', 'HFD_Coeffs_R_Tau_Alpba_Shoulder_Force.m', 'outputs', ...
    {...
    'A11','A12','A13','A14','A15','A16',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_R_Tau_Alpba_Shoulder_HFD(1,7), ...
    'file', 'HFD_Coeffs_R_Tau_Alpba_Shoulder_Constant.m', 'outputs', ...
    {...
    'A17',...
    }});
submit(job)
job.Tasks

coeffs_R_Tau_Beta_Shoulder_HFD = coeffs_Vector(X(6), [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z]);

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_R_Tau_Beta_Shoulder_HFD(1,1), coeffs_R_Tau_Beta_Shoulder_HFD(1,2), coeffs_R_Tau_Beta_Shoulder_HFD(1,3), ...
    coeffs_R_Tau_Beta_Shoulder_HFD(1,4), coeffs_R_Tau_Beta_Shoulder_HFD(1,5), coeffs_R_Tau_Beta_Shoulder_HFD(1,6), ...
    'file', 'HFD_Coeffs_R_Tau_Beta_Shoulder_Force.m', 'outputs', ...
    {...
    'A11','A12','A13','A14','A15','A16',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_R_Tau_Beta_Shoulder_HFD(1,7), ...
    'file', 'HFD_Coeffs_R_Tau_Beta_Shoulder_Constant.m', 'outputs', ...
    {...
    'A17',...
    }});
submit(job)
job.Tasks
%}

% ddr_Beta_Had, ddl_Beta_Hand defined
%{
ddr_Shoulder = formula(diff(r_Shoulder, t, t))';
ddr_Shoulder = subs(ddr_Shoulder, syms_Replaced, syms_Replacing);
ddr_Shoulder = subs(ddr_Shoulder, variables, X');

coeffs_Ddr_Shoulder_HFD = coeffs_Vector(ddr_Shoulder, [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z]);

size(coeffs_Ddr_Shoulder_HFD)

coeffs_Ddr_Shoulder_Force = coeffs_Ddr_Shoulder_HFD(:, 1:6);
coeffs_Ddr_Shoulder_Constant = coeffs_Ddr_Shoulder_HFD(:, 7);

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Shoulder_Force(1,1), coeffs_Ddr_Shoulder_Force(1,2), coeffs_Ddr_Shoulder_Force(1,3), coeffs_Ddr_Shoulder_Force(1,4), coeffs_Ddr_Shoulder_Force(1,5), coeffs_Ddr_Shoulder_Force(1,6), ...
    'file', 'HFD_Coeffs_Ddr_Shoulder_Force_Row1.m', 'outputs', ...
    {...
    'A11','A12','A13','A14','A15','A16',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Shoulder_Force(2,1), coeffs_Ddr_Shoulder_Force(2,2), coeffs_Ddr_Shoulder_Force(2,3), coeffs_Ddr_Shoulder_Force(2,4), coeffs_Ddr_Shoulder_Force(2,5), coeffs_Ddr_Shoulder_Force(2,6), ...
    'file', 'HFD_Coeffs_Ddr_Shoulder_Force_Row2.m', 'outputs', ...
    {...
    'A21','A22','A23','A24','A25','A26',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Shoulder_Force(3,1), coeffs_Ddr_Shoulder_Force(3,2), coeffs_Ddr_Shoulder_Force(3,3), coeffs_Ddr_Shoulder_Force(3,4), coeffs_Ddr_Shoulder_Force(3,5), coeffs_Ddr_Shoulder_Force(3,6), ...
    'file', 'HFD_Coeffs_Ddr_Shoulder_Force_Row3.m', 'outputs', ...
    {...
    'A31','A32','A33','A34','A35','A36',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Shoulder_Constant(1,1), ...
    coeffs_Ddr_Shoulder_Constant(2,1), ...
    coeffs_Ddr_Shoulder_Constant(3,1), ...
    'file', 'HFD_Coeffs_Ddr_Shoulder_Constant.m', 'outputs', ...
    {...
    'A11',...
    'A21',...
    'A31',...
    }});
submit(job)
job.Tasks

ddl_Shoulder = formula(diff(l_Shoulder, t, t))';
ddl_Shoulder = subs(ddl_Shoulder, syms_Replaced, syms_Replacing);
ddl_Shoulder = subs(ddl_Shoulder, variables, X');

coeffs_Ddl_Shoulder_HFD = coeffs_Vector(ddl_Shoulder, [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z]);

size(coeffs_Ddl_Shoulder_HFD)

coeffs_Ddl_Shoulder_Force = coeffs_Ddl_Shoulder_HFD(:, 1:6);
coeffs_Ddl_Shoulder_Constant = coeffs_Ddl_Shoulder_HFD(:, 7);

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddl_Shoulder_Force(1,1), coeffs_Ddl_Shoulder_Force(1,2), coeffs_Ddl_Shoulder_Force(1,3), coeffs_Ddl_Shoulder_Force(1,4), coeffs_Ddl_Shoulder_Force(1,5), coeffs_Ddl_Shoulder_Force(1,6), ...
    'file', 'HFD_Coeffs_Ddl_Shoulder_Force_Row1.m', 'outputs', ...
    {...
    'A11','A12','A13','A14','A15','A16',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddl_Shoulder_Force(2,1), coeffs_Ddl_Shoulder_Force(2,2), coeffs_Ddl_Shoulder_Force(2,3), coeffs_Ddl_Shoulder_Force(2,4), coeffs_Ddl_Shoulder_Force(2,5), coeffs_Ddl_Shoulder_Force(2,6), ...
    'file', 'HFD_Coeffs_Ddl_Shoulder_Force_Row2.m', 'outputs', ...
    {...
    'A21','A22','A23','A24','A25','A26',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddl_Shoulder_Force(3,1), coeffs_Ddl_Shoulder_Force(3,2), coeffs_Ddl_Shoulder_Force(3,3), coeffs_Ddl_Shoulder_Force(3,4), coeffs_Ddl_Shoulder_Force(3,5), coeffs_Ddl_Shoulder_Force(3,6), ...
    'file', 'HFD_Coeffs_Ddl_Shoulder_Force_Row3.m', 'outputs', ...
    {...
    'A31','A32','A33','A34','A35','A36',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddl_Shoulder_Constant(1,1), ...
    coeffs_Ddl_Shoulder_Constant(2,1), ...
    coeffs_Ddl_Shoulder_Constant(3,1), ...
    'file', 'HFD_Coeffs_Ddl_Shoulder_Constant.m', 'outputs', ...
    {...
    'A11',...
    'A21',...
    'A31',...
    }});
submit(job)
job.Tasks
%}

% ddr_Beta_Had, ddl_Beta_Hand defined
%{
ddr_Shoulder = diff(r_Shoulder, t, t);
ddr_Shoulder = subs(ddr_Shoulder, syms_Replaced, syms_Replacing);
ddr_Shoulder = subs(ddr_Shoulder, variables, X');

[coeffs_Ddr_Shoulder_HFD(1, :), ~] = coeffs(ddr_Shoulder(1), [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z, r_Tau_Beta_Shoulder, l_Tau_Beta_Shoulder]);
[coeffs_Ddr_Shoulder_HFD(2, :), ~] = coeffs(ddr_Shoulder(2), [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z, r_Tau_Beta_Shoulder, l_Tau_Beta_Shoulder]);
[coeffs_Ddr_Shoulder_HFD(3, :), ~] = coeffs(ddr_Shoulder(3), [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z, r_Tau_Beta_Shoulder, l_Tau_Beta_Shoulder]);

size(coeffs_Ddr_Shoulder_HFD)

coeffs_Ddr_Shoulder_Force = coeffs_Ddr_Shoulder_HFD(:, 1:6);
coeffs_Ddr_Shoulder_Tau_Beta = coeffs_Ddr_Shoulder_HFD(:, 7:8);
coeffs_Ddr_Shoulder_Constant = coeffs_Ddr_Shoulder_HFD(:, 9);

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Shoulder_Force(1,1), coeffs_Ddr_Shoulder_Force(1,2), coeffs_Ddr_Shoulder_Force(1,3), coeffs_Ddr_Shoulder_Force(1,4), coeffs_Ddr_Shoulder_Force(1,5), coeffs_Ddr_Shoulder_Force(1,6), ...
    coeffs_Ddr_Shoulder_Force(2,1), coeffs_Ddr_Shoulder_Force(2,2), coeffs_Ddr_Shoulder_Force(2,3), coeffs_Ddr_Shoulder_Force(2,4), coeffs_Ddr_Shoulder_Force(2,5), coeffs_Ddr_Shoulder_Force(2,6), ...
    coeffs_Ddr_Shoulder_Force(3,1), coeffs_Ddr_Shoulder_Force(3,2), coeffs_Ddr_Shoulder_Force(3,3), coeffs_Ddr_Shoulder_Force(3,4), coeffs_Ddr_Shoulder_Force(3,5), coeffs_Ddr_Shoulder_Force(3,6), ...
    'file', 'HFD_Coeffs_Ddr_Shoulder_Force.m', 'outputs', ...
    {...
    'A11','A12','A13','A14','A15','A16',...
    'A21','A22','A23','A24','A25','A26',...
    'A31','A32','A33','A34','A35','A36',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Shoulder_Tau_Beta(1,1), coeffs_Ddr_Shoulder_Tau_Beta(1,2), ...
    coeffs_Ddr_Shoulder_Tau_Beta(2,1), coeffs_Ddr_Shoulder_Tau_Beta(2,2), ...
    coeffs_Ddr_Shoulder_Tau_Beta(3,1), coeffs_Ddr_Shoulder_Tau_Beta(3,2), ...
    'file', 'HFD_Coeffs_Ddr_Shoulder_Tau_Beta.m', 'outputs', ...
    {...
    'A11','A12',...
    'A21','A22',...
    'A31','A32',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Shoulder_Constant(1,1), ...
    coeffs_Ddr_Shoulder_Constant(2,1), ...
    coeffs_Ddr_Shoulder_Constant(3,1), ...
    'file', 'HFD_Coeffs_Ddr_Shoulder_Constant.m', 'outputs', ...
    {...
    'A11',...
    'A21',...
    'A31',...
    }});
submit(job)
job.Tasks

ddl_Shoulder = diff(l_Shoulder, t, t);
ddl_Shoulder = subs(ddl_Shoulder, syms_Replaced, syms_Replacing);
ddl_Shoulder = subs(ddl_Shoulder, variables, X');

[coeffs_Ddl_Shoulder_HFD(1, :), ~] = coeffs(ddl_Shoulder(1), [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z, r_Tau_Beta_Shoulder, l_Tau_Beta_Shoulder]);
[coeffs_Ddl_Shoulder_HFD(2, :), ~] = coeffs(ddl_Shoulder(2), [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z, r_Tau_Beta_Shoulder, l_Tau_Beta_Shoulder]);
[coeffs_Ddl_Shoulder_HFD(3, :), ~] = coeffs(ddl_Shoulder(3), [r_F_X, r_F_Y, r_F_Z, l_F_X, l_F_Y, l_F_Z, r_Tau_Beta_Shoulder, l_Tau_Beta_Shoulder]);

size(coeffs_Ddl_Shoulder_HFD)

coeffs_Ddl_Shoulder_Force = coeffs_Ddl_Shoulder_HFD(:, 1:6);
coeffs_Ddl_Shoulder_Tau_Beta = coeffs_Ddl_Shoulder_HFD(:, 7:8);
coeffs_Ddl_Shoulder_Constant = coeffs_Ddl_Shoulder_HFD(:, 9);

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddl_Shoulder_Force(1,1), coeffs_Ddl_Shoulder_Force(1,2), coeffs_Ddl_Shoulder_Force(1,3), coeffs_Ddl_Shoulder_Force(1,4), coeffs_Ddl_Shoulder_Force(1,5), coeffs_Ddl_Shoulder_Force(1,6), ...
    coeffs_Ddl_Shoulder_Force(2,1), coeffs_Ddl_Shoulder_Force(2,2), coeffs_Ddl_Shoulder_Force(2,3), coeffs_Ddl_Shoulder_Force(2,4), coeffs_Ddl_Shoulder_Force(2,5), coeffs_Ddl_Shoulder_Force(2,6), ...
    coeffs_Ddl_Shoulder_Force(3,1), coeffs_Ddl_Shoulder_Force(3,2), coeffs_Ddl_Shoulder_Force(3,3), coeffs_Ddl_Shoulder_Force(3,4), coeffs_Ddl_Shoulder_Force(3,5), coeffs_Ddl_Shoulder_Force(3,6), ...
    'file', 'HFD_Coeffs_Ddl_Shoulder_Force.m', 'outputs', ...
    {...
    'A11','A12','A13','A14','A15','A16',...
    'A21','A22','A23','A24','A25','A26',...
    'A31','A32','A33','A34','A35','A36',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddl_Shoulder_Tau_Beta(1,1), coeffs_Ddl_Shoulder_Tau_Beta(1,2), ...
    coeffs_Ddl_Shoulder_Tau_Beta(2,1), coeffs_Ddl_Shoulder_Tau_Beta(2,2), ...
    coeffs_Ddl_Shoulder_Tau_Beta(3,1), coeffs_Ddl_Shoulder_Tau_Beta(3,2), ...
    'file', 'HFD_Coeffs_Ddl_Shoulder_Tau_Beta.m', 'outputs', ...
    {...
    'A11','A12',...
    'A21','A22',...
    'A31','A32',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddl_Shoulder_Constant(1,1), ...
    coeffs_Ddl_Shoulder_Constant(2,1), ...
    coeffs_Ddl_Shoulder_Constant(3,1), ...
    'file', 'HFD_Coeffs_Ddl_Shoulder_Constant.m', 'outputs', ...
    {...
    'A11',...
    'A21',...
    'A31',...
    }});
submit(job)
job.Tasks
%}
%}

%% Full Reverse dynamics
%{
ddr_Shoulder = diff(r_Shoulder, t, t);
ddr_Shoulder = subs(ddr_Shoulder, syms_Replaced, syms_Replacing);
ddl_Shoulder = diff(l_Shoulder, t, t);
ddl_Shoulder = subs(ddl_Shoulder, syms_Replaced, syms_Replacing);

job = createJob(c);
createTask(job, @matlabFunction, 1,{ddr_Shoulder, ...
    'file', 'FRD_Ddr_Shoulder.m', 'outputs', ...
    {'ddr_Shoulder'}});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{ddl_Shoulder, ...
    'file', 'FRD_Ddl_Shoulder.m', 'outputs', ...
    {'ddl_Shoulder'}});
submit(job)
job.Tasks

dr_Shoulder = diff(r_Shoulder, t);
dr_Shoulder = subs(dr_Shoulder, syms_Replaced, syms_Replacing);
dl_Shoulder = diff(l_Shoulder, t);
dl_Shoulder = subs(dl_Shoulder, syms_Replaced, syms_Replacing);

job = createJob(c);
createTask(job, @matlabFunction, 1,{dr_Shoulder, ...
    'file', 'FRD_Dr_Shoulder.m', 'outputs', ...
    {'dr_Shoulder'}});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{dl_Shoulder, ...
    'file', 'FRD_Dl_Shoulder.m', 'outputs', ...
    {'dl_Shoulder'}});
submit(job)
job.Tasks

r_Shoulder = subs(r_Shoulder, syms_Replaced, syms_Replacing);
l_Shoulder = subs(l_Shoulder, syms_Replaced, syms_Replacing);

job = createJob(c);
createTask(job, @matlabFunction, 1,{l_Shoulder, ...
    'file', 'FRD_L_Shoulder.m', 'outputs', ...
    {'l_Shoulder'}});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{r_Shoulder, ...
    'file', 'FRD_R_Shoulder.m', 'outputs', ...
    {'r_Shoulder'}});
submit(job)
job.Tasks

r_Hip = subs(r_Hip, syms_Replaced, syms_Replacing);
l_Hip = subs(l_Hip, syms_Replaced, syms_Replacing);

job = createJob(c);
createTask(job, @matlabFunction, 1,{l_Hip, ...
    'file', 'FRD_L_Hip.m', 'outputs', ...
    {'l_Hip'}});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{r_Hip, ...
    'file', 'FRD_R_Hip.m', 'outputs', ...
    {'r_Hip'}});
submit(job)
job.Tasks
%}



































