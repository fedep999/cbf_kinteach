% ==============================================================================
% Franka Emika Panda — Direct Joint-Torque CBF Simulation Initialization
% ==============================================================================

% clearvars -except FASTEXECUTION;
close all;

% Ensure MEX binary is on path
addpath(fileparts(mfilename('fullpath')));

FASTEXECUTION = false;

% Import Panda robot model from local RobotFiles symlink
Franka_robot = importrobot("RobotFiles/Franka_Panda/Franka_arm.urdf");
Franka_robot.DataFormat = 'column';
Franka_robot.Gravity = [0 0 -9.81];

% Nullspace Posture & Damping
Kn = 0 * ones(7,1); 
Dn = 20 * ones(7,1);

% Initial and Home Configuration for simulation
% q_init = [0 -pi/4 0 -(3/4)*pi 0 pi/2 pi/4]';
q_init = [0 0  0  -(2/4)*pi  0  pi/2  -pi/4]'; 
% q_home = [0.003542735427650; -0.076954344513307; -0.003006835131443; ...
%           -1.362053944077202; -5.624494697578509e-04; 1.279919277568267; 0.786445509714290];
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
% DIRECT JOINT-TORQUE CONTROL BARRIER FUNCTION (CBF) HYPERPARAMETERS
% Links directly to C++ formulation in torque_cbf::TorqueCBFController
% ==============================================================================
xTableHeight = 0.106 - 0.004; % Table/barrier surface height [m] (0.102 m)

plane_normal = [0.0; 0.0; 1.0];                   % Hyperplane normal n_hp
plane_offset = [0.48; 0.16; xTableHeight];        % Hyperplane offset

cbf_p_ths     = 0.30;    % Distance activation threshold [m] (p_ths > 0)
cbf_v_ths     = -0.003;  % Terminal contact velocity threshold [m/s] (v_ths < 0)
cbf_v_0       = -0.80;   % Nominal entry approach velocity [m/s] (v_0 < v_ths < 0)
cbf_gamma     = 12.0;    % Class-K decay gain [1/s]
cbf_w_lin     = 1e6;     % Slack linear penalty weight
cbf_w_slack   = 1e4;     % Slack quadratic penalty weight
cbf_tau_limit = 150.0;   % Legacy scalar limit [Nm] (fallback only, superseded by cbf_tau_max)
% Franka Emika Panda joint torque limits [Nm] (datasheet / libfranka tau_max).
% Enforced on the TOTAL torque g + tau_null + tau_cbf.
cbf_tau_max   = [87; 87; 87; 87; 12; 12; 12];
cbf_dt        = 0.001;   % Sampling time [s] for passivity / one-way braking cap

% Lateral spring-damper (add_lateral_stiffness.m): Cartesian spring towards H_home plus
% damping with constant damping ratio, tau_stiff = -Jg'*(Fk + Fd),
% D_i = 2*zeta_i*sqrt(K_i*Lambda_ii). Keep the z entries 0 (free along the plane normal).
cbf_Kd_trans   = [2000; 2000; 0]*0;  % Translational stiffness x, y, z [N/m]
cbf_Kd_rot     = [120; 120; 120]*0;  % Rotational stiffness [Nm/rad] (holds orientation)
cbf_zeta_trans = [0.7; 0.7; 0]*0;    % Translational damping ratio x, y, z [-] (1 = critical)
cbf_zeta_rot   = [0.7; 0.7; 0.7]*0;  % Rotational damping ratio [-] (0 = undamped orientation spring)
% 1 = spring-damper dynamically decoupled from the plane normal: it produces zero
% acceleration along n (only the CBF acts along the normal). 0 = plain spring-damper.
cbf_stiff_decouple = 0;   % 1 made the robot drift/oscillate along z when untouched (9 Nov 2026), keep 0

% CBF_Logger (add_cbf_logger.m / load_cbf_log.m): 49 columns per logged sample
% (~0.4 MB/s at 1 kHz). Increase to log every n-th sample only.
cbf_log_decimation = 1;

% Entry-velocity latch (one-sided): entering the zone FASTER than v_0 restarts the
% envelope from the entry speed (smooth onset, B = 0 at entry); slower entries keep v_0
% (no CBF action at entry).
cbf_latch_entry  = 1;      % 1 = enabled, 0 = disabled
cbf_zone_hyst    = 0.005;  % Zone exit hysteresis [m]: no re-latch when hovering around p_ths
cbf_latch_alpha  = 0.2;    % EMA coefficient (0,1] of the latched velocity (0.2 @ 1 kHz ~ 5 ms)

cbf_tau_rate = 0;     % [Nm/s] CBF torque-rate budget (0 = off, recommended)

% Packed parameters for sfun_torque_cbf (25x1):
% [p_ths, v_ths, v_0, gamma, w_lin, w_slack, tau_limit, dt, nx, ny, nz, ox, oy, oz,
%  latch_entry_velocity, zone_exit_hysteresis, latch_velocity_filter_alpha,
%  tau_max_1 ... tau_max_7, tau_rate_limit]
cbf_torque_params = [cbf_p_ths; cbf_v_ths; cbf_v_0; cbf_gamma; cbf_w_lin; cbf_w_slack; ...
                     cbf_tau_limit; cbf_dt; plane_normal; plane_offset; ...
                     cbf_latch_entry; cbf_zone_hyst; cbf_latch_alpha; cbf_tau_max; cbf_tau_rate];
assert(numel(cbf_torque_params) == 25, 'cbf_torque_params must be 25x1 (see sfun_torque_cbf.cpp)');
% Savitzky-Golay force filter (aggressive)
sg_p    = 2;        % polynomial order: 1 = strongest smoothing, 2 = keeps peaks better
sg_N    = 21;      % window length (odd): 101 samples = 100 ms span at 1 kHz
sg_mode = 'causal';   % 'centered' or 'causal'

B = sgolay(sg_p, sg_N);                 % N x N; row i estimates sample i of the window
switch sg_mode
    case 'centered'
        b_sg = B((sg_N+1)/2, :);        % symmetric; delay = (N-1)/2 samples
    case 'causal'
        b_sg = fliplr(B(end, :));       % endpoint estimate; b_sg(1) * newest sample
end

fprintf('SG filter: N=%d, p=%d, %s, noise gain = %.4f, delay = %g ms\n', ...
        sg_N, sg_p, sg_mode, sum(b_sg.^2), ...
        strcmp(sg_mode,'centered') * (sg_N-1)/2 * 1e3 * 0.001);
disp('Torque CBF Simulation Initialized successfully.');
