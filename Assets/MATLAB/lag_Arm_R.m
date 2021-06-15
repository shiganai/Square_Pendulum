
clear all
tic

parallel.defaultClusterProfile('local');
c = parcluster();

syms width_Body real

syms m_Hand real
syms length_Hand real
syms g real

syms r_X_Fixed r_Y_Fixed r_Z_Fixed real
syms r_F_X r_F_Y r_F_Z real
syms r_Tau_Alpha_Shoulder real
syms r_Tau_Beta_Shoulder real

syms r_Alpha_Hand_Pre(t)
syms r_Beta_Hand_Pre(t)

%%
syms r_Alpha_Hand dr_Alpha_Hand ddr_Alpha_Hand real
syms r_Beta_Hand dr_Beta_Hand ddr_Beta_Hand real

syms_Replaced = [
    r_Alpha_Hand_Pre diff(r_Alpha_Hand_Pre, t) diff(r_Alpha_Hand_Pre, t, t), ...
    r_Beta_Hand_Pre diff(r_Beta_Hand_Pre, t) diff(r_Beta_Hand_Pre, t, t), ...
    ];

syms_Replacing = [
    r_Alpha_Hand dr_Alpha_Hand ddr_Alpha_Hand ...
    r_Beta_Hand dr_Beta_Hand ddr_Beta_Hand ...
    ];

%%
I_Hand = 1/12 * m_Hand * [
    length_Hand^2 + 0^2, 0, 0;
    0, 0, 0^2 + 0^2;
    0, 0^2 + length_Hand^2, 0;
    ];

% I_Hand = 1/12 * m_Hand * [
%     length_Hand^2 + radius_Hand^2, 0, 0;
%     0, radius_Hand^2 + length_Hand^2, 0;
%     0, 0, radius_Hand^2 + radius_Hand^2;
%     ];

%%

r_Arm_Bottom = [0, 0+ length_Hand, 0];
r_Arm_G = [0, 0 + length_Hand/2, 0];

%{
%% rotate beta around z
r_Tauvec_Beta = symfun([0, 0, 1, 1], t);
r_Trans_Matrix_Beta = [cos(r_Beta_Hand_Pre), -sin(r_Beta_Hand_Pre), 0, 0; sin(r_Beta_Hand_Pre), cos(r_Beta_Hand_Pre), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1]';

r_Arm_Bottom = r_Arm_Bottom * r_Trans_Matrix_Beta;
r_Arm_G = r_Arm_G * r_Trans_Matrix_Beta;

%% rotate alpha around x
r_Tauvec_Alpha = symfun([1, 0, 0, 1], t);
r_Trans_Matrix_Alpha = [1, 0, 0, 0; 0, cos(r_Alpha_Hand_Pre), -sin(r_Alpha_Hand_Pre), 0; 0, sin(r_Alpha_Hand_Pre), cos(r_Alpha_Hand_Pre), 0; 0, 0, 0, 1]';

r_Arm_Bottom = r_Arm_Bottom * r_Trans_Matrix_Alpha;
r_Arm_G = r_Arm_G * r_Trans_Matrix_Alpha;

r_Tauvec_Beta = r_Tauvec_Beta * r_Trans_Matrix_Alpha;

%% move origin
r_Trans_Matrix_Origin = [1, 0, 0, r_X_Fixed; 0, 1, 0, r_Y_Fixed; 0, 0, 1, r_Z_Fixed; 0, 0, 0, 1]';

r_Arm_Bottom = r_Arm_Bottom * r_Trans_Matrix_Origin;
r_Arm_G = r_Arm_G * r_Trans_Matrix_Origin;
%}

%{/
%% Rotate
r_Rotate_Matrix = ...
    [cos(r_Beta_Hand_Pre), -sin(r_Beta_Hand_Pre), 0; sin(r_Beta_Hand_Pre), cos(r_Beta_Hand_Pre), 0; 0, 0, 1;]' ...
    * ...
    [1, 0, 0; 0, cos(r_Alpha_Hand_Pre), -sin(r_Alpha_Hand_Pre); 0, sin(r_Alpha_Hand_Pre), cos(r_Alpha_Hand_Pre);]' ...
    * ...
    1;

dr_Rotate_Matrix = diff(r_Rotate_Matrix, t);

r_Arm_Bottom = r_Arm_Bottom * r_Rotate_Matrix;
r_Arm_G = r_Arm_G * r_Rotate_Matrix;

%% Move
r_Arm_Bottom = r_Arm_Bottom + [r_X_Fixed, r_Y_Fixed, r_Z_Fixed];
r_Arm_G = r_Arm_G + [r_X_Fixed, r_Y_Fixed, r_Z_Fixed];

%% 
ohm_R = formula(r_Rotate_Matrix' * dr_Rotate_Matrix);

omega_X = ohm_R(2,3);
omega_Y = ohm_R(3,1);
omega_Z = ohm_R(1,2);

omega_R = simplify([omega_X; omega_Y; omega_Z]);
omega_Euler_R = simplify(r_Rotate_Matrix * omega_R);

omega_R_Tmp = simplify(subs(omega_R, syms_Replaced, syms_Replacing));
unit_Vector_R = coeffs_Vector(omega_R_Tmp, [dr_Alpha_Hand, dr_Beta_Hand]);
unit_Vector_R = subs(unit_Vector_R, syms_Replacing, syms_Replaced);

%}

%%

r_Arm_Bottom = formula(r_Arm_Bottom);
r_Arm_G = formula(r_Arm_G);
% r_Tauvec_Alpha = formula(r_Tauvec_Alpha);
% r_Tauvec_Beta = formula(r_Tauvec_Beta);

r_V_G_Hand = diff(r_Arm_G, t);

T = ...
    1/2 * m_Hand * (r_V_G_Hand * r_V_G_Hand')...
    + ...
    1/2 * (omega_Euler_R' * (I_Hand * omega_Euler_R))...
    + ...
    0;

U = ...
    m_Hand * g * r_Arm_G(3)...
    +...
    0;

L = T - U;

r_Tau_Vec = (-r_Tau_Alpha_Shoulder) * unit_Vector_R(:,1) + (-r_Tau_Beta_Shoulder) * unit_Vector_R(:,2);

coeffs_Tau_R_Alpha = unit_Vector_R(:,1)' * r_Tau_Vec;
coeffs_Tau_R_Beta = unit_Vector_R(:,2)' * r_Tau_Vec;

%%

equations = [
    -functionalDerivative(L, r_Alpha_Hand_Pre) == coeffs_Tau_R_Alpha - ([r_F_X, r_F_Y, r_F_Z] * diff(r_Arm_Bottom, r_Alpha_Hand_Pre)');
    -functionalDerivative(L, r_Beta_Hand_Pre) == coeffs_Tau_R_Beta - ([r_F_X, r_F_Y, r_F_Z] * diff(r_Arm_Bottom, r_Beta_Hand_Pre)');
    ];
equations = subs(equations, syms_Replaced, syms_Replacing);

%% Full forward dynamics
%{
variables = [ddr_Alpha_Hand, ddr_Beta_Hand];

[A, B] = equationsToMatrix(equations, variables);
toc
tic
X = inv(A)*B;
toc

parallel.defaultClusterProfile('local');
c = parcluster();

job = createJob(c);
createTask(job, @matlabFunction, 1,{X(1), X(2), ...
    'file', 'FFD_Dds_Arm_R.m', 'outputs', ...
    {'ddr_Alpha_Hand', 'ddr_Beta_Hand'}});
submit(job)
job.Tasks

ddr_Arm_Bottom = diff(r_Arm_Bottom, t, t);
ddr_Arm_Bottom = subs(ddr_Arm_Bottom, syms_Replaced, syms_Replacing);
ddr_Arm_Bottom = subs(ddr_Arm_Bottom, variables, X');

[coeffs_Ddr_Arm_Bottom(1, :), ~] = coeffs(ddr_Arm_Bottom(1), [r_F_X, r_F_Y, r_F_Z]);

[coeffs_Ddr_Arm_Bottom(2, :), ~] = coeffs(ddr_Arm_Bottom(2), [r_F_X, r_F_Y, r_F_Z]);

[coeffs_Ddr_Arm_Bottom(3, :), ~] = coeffs(ddr_Arm_Bottom(3), [r_F_X, r_F_Y, r_F_Z]);

size(coeffs_Ddr_Arm_Bottom)

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Arm_Bottom(1,1), coeffs_Ddr_Arm_Bottom(1,2), coeffs_Ddr_Arm_Bottom(1,3), coeffs_Ddr_Arm_Bottom(1,4), ...
    coeffs_Ddr_Arm_Bottom(2,1), coeffs_Ddr_Arm_Bottom(2,2), coeffs_Ddr_Arm_Bottom(2,3), coeffs_Ddr_Arm_Bottom(2,4), ...
    coeffs_Ddr_Arm_Bottom(3,1), coeffs_Ddr_Arm_Bottom(3,2), coeffs_Ddr_Arm_Bottom(3,3), coeffs_Ddr_Arm_Bottom(3,4), ...
    'file', 'FFD_Coeffs_Ddr_Arm_Bottom.m', 'outputs', ...
    {...
    'A11','A12','A13','A14',...
    'A21','A22','A23','A24',...
    'A31','A32','A33','A34',...
    }});
submit(job)
job.Tasks
%}

%% Half forward dynamics
%{/
variables = [ddr_Alpha_Hand, ddr_Beta_Hand];

[A, B] = equationsToMatrix(equations, variables);
toc
tic
X = inv(A)*B;
toc

% job = createJob(c);
% createTask(job, @matlabFunction, 1,{X(1), X(2), ...
%     'file', 'HFD_Dds_Arm_R.m', 'outputs', ...
%     {'ddr_Alpha_Hand', 'r_Tau_Beta_Shoulder'}});
% submit(job)
% job.Tasks

ddr_Arm_Bottom = formula(diff(r_Arm_Bottom, t, t))';
ddr_Arm_Bottom = subs(ddr_Arm_Bottom, syms_Replaced, syms_Replacing);
ddr_Arm_Bottom = subs(ddr_Arm_Bottom, variables, X');

coeffs_Ddr_Arm_Bottom = coeffs_Vector(ddr_Arm_Bottom, [r_F_X, r_F_Y, r_F_Z, r_Tau_Alpha_Shoulder, r_Tau_Beta_Shoulder]);

size(coeffs_Ddr_Arm_Bottom)

coeffs_Ddr_Arm_Bottom_Force = coeffs_Ddr_Arm_Bottom(:, 1:3);
coeffs_Ddr_Arm_Bottom_Tau = coeffs_Ddr_Arm_Bottom(:, 4:5);
coeffs_Ddr_Arm_Bottom_Constant = coeffs_Ddr_Arm_Bottom(:, 6);

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Arm_Bottom_Force(1,1), coeffs_Ddr_Arm_Bottom_Force(1,2), coeffs_Ddr_Arm_Bottom_Force(1,3), ...
    coeffs_Ddr_Arm_Bottom_Force(2,1), coeffs_Ddr_Arm_Bottom_Force(2,2), coeffs_Ddr_Arm_Bottom_Force(2,3), ...
    coeffs_Ddr_Arm_Bottom_Force(3,1), coeffs_Ddr_Arm_Bottom_Force(3,2), coeffs_Ddr_Arm_Bottom_Force(3,3), ...
    'file', 'HFD_Coeffs_Ddr_Arm_Bottom_Force.m', 'outputs', ...
    {...
    'A11','A12','A13',...
    'A21','A22','A23',...
    'A31','A32','A33',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Arm_Bottom_Tau(1,1), coeffs_Ddr_Arm_Bottom_Tau(1,2), ...
    coeffs_Ddr_Arm_Bottom_Tau(2,1), coeffs_Ddr_Arm_Bottom_Tau(2,2), ...
    coeffs_Ddr_Arm_Bottom_Tau(3,1), coeffs_Ddr_Arm_Bottom_Tau(3,2), ...
    'file', 'HFD_Coeffs_Ddr_Arm_Bottom_Tau.m', 'outputs', ...
    {...
    'A11','A12',...
    'A21','A22',...
    'A31','A32',...
    }});
submit(job)
job.Tasks

job = createJob(c);
createTask(job, @matlabFunction, 1,{...
    coeffs_Ddr_Arm_Bottom_Constant(1,1), ...
    coeffs_Ddr_Arm_Bottom_Constant(2,1), ...
    coeffs_Ddr_Arm_Bottom_Constant(3,1), ...
    'file', 'HFD_Coeffs_Ddr_Arm_Bottom_Constant.m', 'outputs', ...
    {...
    'A11',...
    'A21',...
    'A31',...
    }});
submit(job)
job.Tasks
%}

%% Full Reverse Dynamics
%{
ddr_Arm_Bottom = diff(r_Arm_Bottom, t, t);
ddr_Arm_Bottom = subs(ddr_Arm_Bottom, syms_Replaced, syms_Replacing);

job = createJob(c);
createTask(job, @matlabFunction, 1,{ddr_Arm_Bottom, ...
    'file', 'FRD_Ddr_Arm_Bottom.m', 'outputs', ...
    {'ddr_Arm_Bottom'}});
submit(job)
job.Tasks

dr_Arm_Bottom = diff(r_Arm_Bottom, t);
dr_Arm_Bottom = subs(dr_Arm_Bottom, syms_Replaced, syms_Replacing);

job = createJob(c);
createTask(job, @matlabFunction, 1,{dr_Arm_Bottom, ...
    'file', 'FRD_Dr_Arm_Bottom.m', 'outputs', ...
    {'dr_Arm_Bottom'}});
submit(job)
job.Tasks

r_Arm_Bottom = subs(r_Arm_Bottom, syms_Replaced, syms_Replacing);

job = createJob(c);
createTask(job, @matlabFunction, 1,{r_Arm_Bottom, ...
    'file', 'FRD_R_Arm_Bottom.m', 'outputs', ...
    {'r_Arm_Bottom'}});
submit(job)
job.Tasks
%}





























