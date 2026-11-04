% ==============================================================================
% Franka Emika Panda — Direct Joint-Torque CBF Simulation Initialization
% ==============================================================================

clearvars -except FASTEXECUTION;
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
q_init = [0 -pi/4 0 -(3/4)*pi 0 pi/2 pi/4]';
q_home = [0.003542735427650; -0.076954344513307; -0.003006835131443; ...
          -1.362053944077202; -5.624494697578509e-04; 1.279919277568267; 0.786445509714290];

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
cbf_v_0       = -0.40;   % Nominal entry approach velocity [m/s] (v_0 < v_ths < 0)
cbf_gamma     = 2.0;    % Class-K decay gain [1/s]
cbf_w_lin     = 1e6;     % Slack linear penalty weight
cbf_w_slack   = 1e4;     % Slack quadratic penalty weight
cbf_tau_limit = 150.0;   % Joint torque limit [Nm]
cbf_dt        = 0.001;   % Sampling time [s] for passivity / one-way braking cap

% Packed parameters for sfun_torque_cbf:
% [p_ths, v_ths, v_0, gamma, w_lin, w_slack, tau_limit, dt, nx, ny, nz, ox, oy, oz]
cbf_torque_params = [cbf_p_ths; cbf_v_ths; cbf_v_0; cbf_gamma; cbf_w_lin; cbf_w_slack; ...
                     cbf_tau_limit; cbf_dt; plane_normal; plane_offset];

disp('Torque CBF Simulation Initialized successfully.');
