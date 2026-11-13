% ==============================================================================
% Franka Emika Panda — Joint-Space Torque CBF (jtorque_cbf) Initialization
% Model: Controller_jtorque_cbf_22Franka.slx (S-function sfun_joint_torque_cbf)
% Commanded torque: tau = g (added by the robot) + tau_null + tau_stiff + tau_cbf
% ==============================================================================

% clearvars -except FASTEXECUTION;
close all;

% MEX binary and S-function source (needed by the Franka code generation) on the path
this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, 'src'));

FASTEXECUTION = false;

% Import Panda robot model from local RobotFiles symlink
Franka_robot = importrobot(fullfile(this_dir, "RobotFiles/Franka_Panda/Franka_arm.urdf"));
Franka_robot.DataFormat = 'column';
Franka_robot.Gravity = [0 0 -9.81];

% Nullspace Posture & Damping
Kn = 0 * ones(7,1);
Dn = 0 * ones(7,1);

% Initial and Home Configuration
q_init = [0 0  0  -(2/4)*pi  0  pi/2  -pi/4]';
q_home = q_init;

H_home = getTransform(Franka_robot, q_home, 'panda_link8', 'panda_link0');

% Rotations
Rz = @(x) [cos(x), -sin(x), 0; sin(x), cos(x), 0; 0, 0, 1];
Ry = @(x) [cos(x), 0, sin(x); 0, 1, 0; -sin(x), 0, cos(x)];
Rx = @(x) [1, 0, 0; 0, cos(x), -sin(x); 0, sin(x), cos(x)];

% Reference frame transformations
H_0_pr = [eye(3), [0.5596; 0.1927; -0.07353]; zeros(1,3), 1];
H_pr_cy = [eye(3), [0; -0.2015; 0.1355]; zeros(1,3), 1];
H_0_cy = H_0_pr * H_pr_cy;
H_0_cy_INV = [transpose(H_0_cy(1:3,1:3)), -transpose(H_0_cy(1:3,1:3))*H_0_cy(1:3,4); zeros(1,3), 1];

% ==============================================================================
% BARRIER (same meaning as the torque CBF, see init_torque_cbf.m)
% ==============================================================================
xTableHeight = 0.106 - 0.004; % Table/barrier surface height [m] (0.102 m)

plane_normal = [0.0; 0.0; 1.0];                   % Hyperplane normal n_hp
plane_offset = [0.48; 0.16; xTableHeight];        % Hyperplane offset

cbf_p_ths     = 0.30;    % Distance activation threshold [m] (p_ths > 0)
cbf_v_ths     = -0.003;  % Terminal contact velocity threshold [m/s] (v_ths < 0)
cbf_v_0       = -0.20;   % Nominal entry approach velocity [m/s] (v_0 < v_ths < 0)
cbf_gamma     = 15.0;    % Class-K decay gain [1/s]
cbf_w_lin     = 1e4;     % CBF slack linear penalty weight
cbf_w_slack   = 1e3;     % CBF slack quadratic penalty weight
cbf_tau_limit = 150.0;   % Legacy scalar limit [Nm] (fallback only, superseded by cbf_tau_max)
% Franka Emika Panda joint torque limits [Nm], enforced on the TOTAL torque g + tau_nom + tau_cbf
cbf_tau_max   = [87; 87; 87; 87; 12; 12; 12];
cbf_dt        = 0.001;   % Control period [s]

% Entry-velocity latch (one-sided), see init_torque_cbf.m
cbf_latch_entry  = 0;      % 1 = enabled, 0 = disabled
cbf_zone_hyst    = 0.005;  % Zone exit hysteresis [m]
cbf_latch_alpha  = 0.2;    % EMA coefficient (0,1] of the latched velocity

% ==============================================================================
% JOINT-SPACE TORQUE CBF OPTIONS (jtorque_cbf::JointTorqueCBFConfig)
% ==============================================================================
% Robot torque-rate limit on tau_nom + tau_cbf [Nm/s]. Must equal the Rate Limiter in front of
% 'Apply Control' (999 Nm/s): the QP then plans exactly the torque the robot applies. 0 = off.
cbf_tau_rate      = 999;
% Allowed lateral force / moment of the CBF torque (each component). 0 / 0 = pure normal force
% (identical to the torque CBF). Larger values let the CBF build braking force faster under the
% rate limit, at the cost of a bounded lateral force while braking.
cbf_max_lat_force = 5.0;   % [N]
cbf_max_moment    = 0.5;   % [Nm]
% Cost weights of the end-effector force produced by tau_cbf (normal component weight = 1)
cbf_w_lateral     = 1e4;
cbf_w_moment      = 1e4;
cbf_w_null        = 1e5;   % nullspace torque (no effect on the elbow)
% Legacy compatibility parameter (unused: guarantees are hard when tau = 0 is reachable)
cbf_guar_factor   = 1e3;

% Packed parameters for sfun_joint_torque_cbf (31x1):
% [p_ths, v_ths, v_0, gamma, w_lin, w_slack, tau_limit, dt, nx, ny, nz, ox, oy, oz,
%  latch_entry_velocity, zone_exit_hysteresis, latch_velocity_filter_alpha,
%  tau_max_1 ... tau_max_7, tau_rate_limit, max_lateral_force, max_moment,
%  w_lateral, w_moment, w_null, guarantee_slack_factor]
cbf_jtorque_params = [cbf_p_ths; cbf_v_ths; cbf_v_0; cbf_gamma; cbf_w_lin; cbf_w_slack; ...
                      cbf_tau_limit; cbf_dt; plane_normal; plane_offset; ...
                      cbf_latch_entry; cbf_zone_hyst; cbf_latch_alpha; cbf_tau_max; ...
                      cbf_tau_rate; cbf_max_lat_force; cbf_max_moment; ...
                      cbf_w_lateral; cbf_w_moment; cbf_w_null; cbf_guar_factor];
assert(numel(cbf_jtorque_params) == 31, 'cbf_jtorque_params must be 31x1 (see sfun_joint_torque_cbf.cpp)');

% ==============================================================================
% OTHER MODEL BLOCKS (inherited from the torque CBF model)
% ==============================================================================
% Lateral spring-damper (Lateral_Stiffness block), tau_stiff = -Jg'*(Fk + Fd); zero = off
cbf_Kd_trans   = [2000; 2000; 0]*0;  % Translational stiffness x, y, z [N/m]
cbf_Kd_rot     = [120; 120; 120]*0;  % Rotational stiffness [Nm/rad]
cbf_zeta_trans = [0.7; 0.7; 0]*0;    % Translational damping ratio [-]
cbf_zeta_rot   = [0.7; 0.7; 0.7]*0;  % Rotational damping ratio [-]
cbf_stiff_decouple = 0;              % keep 0 (see init_torque_cbf.m)

% CBF_Logger decimation (if the logger is present in the model)
cbf_log_decimation = 1;

% Savitzky-Golay force filter
sg_p    = 2;          % polynomial order
sg_N    = 21;         % window length (odd)
sg_mode = 'causal';   % 'centered' or 'causal'

B = sgolay(sg_p, sg_N);
switch sg_mode
    case 'centered'
        b_sg = B((sg_N+1)/2, :);
    case 'causal'
        b_sg = fliplr(B(end, :));
end

fprintf('SG filter: N=%d, p=%d, %s, noise gain = %.4f, delay = %g ms\n', ...
        sg_N, sg_p, sg_mode, sum(b_sg.^2), ...
        strcmp(sg_mode,'centered') * (sg_N-1)/2 * 1e3 * 0.001);
disp('Joint-space torque CBF initialized successfully.');
