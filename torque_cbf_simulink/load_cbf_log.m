function L = load_cbf_log(src, do_plot)
% LOAD_CBF_LOG Unpacks the CBF_Logger data (see add_cbf_logger.m) and plots it.
%
%   L = load_cbf_log()                 % reads Controller_trorque_cbf_22Franka.mat
%   L = load_cbf_log('run1.mat')       % reads a given MAT file
%   L = load_cbf_log(cbf_log)          % uses an array already in the workspace
%   L = load_cbf_log(..., false)       % no plots
%
% Returns a struct with one field per signal (rows = samples).
% Convention: F_ext fields are the wrench ON the robot as fed to / estimated for the CBF,
% wrench_raw is the robot's O_F_ext_hat_K as received.

if nargin < 1 || isempty(src)
    src = 'Controller_trorque_cbf_22Franka.mat';
end
if nargin < 2
    do_plot = true;
end

if ischar(src) || isstring(src)
    S = load(src);
    if isfield(S, 'rt_cbf_log')
        D = S.rt_cbf_log;          % MAT-file logging on the robot (rt_ prefix)
    elseif isfield(S, 'cbf_log')
        D = S.cbf_log;
    else
        error('load_cbf_log:novar', 'No rt_cbf_log / cbf_log variable in %s.', src);
    end
else
    D = src;
end
if isstruct(D)                     % 'Structure' save formats
    D = D.signals.values;
end

% Column layout (must match add_cbf_logger.m)
layout = {
    't',          1
    'telemetry',  8
    'F_ext_cbf',  6
    'use_fext',   1
    'F_ext_est',  6
    'wrench_raw', 6
    'tau_cbf',    7
    'tau_stiff',  7
    'tau_null',   7
};
ncol = sum([layout{:,2}]);
if size(D, 2) ~= ncol
    error('load_cbf_log:layout', 'Expected %d columns, got %d.', ncol, size(D, 2));
end
c = 0;
for k = 1:size(layout, 1)
    L.(layout{k,1}) = D(:, c+1 : c+layout{k,2});
    c = c + layout{k,2};
end
tl = L.telemetry;
L.is_active = tl(:,1); L.zeta = tl(:,2); L.dzeta = tl(:,3); L.v_min = tl(:,4);
L.B = tl(:,5); L.f_n = tl(:,6); L.slack = tl(:,7); L.qp_cost = tl(:,8);
L.t = L.t - L.t(1);

if ~do_plot
    return;
end
t = L.t;
figure('Name', 'CBF log', 'Color', 'w');
ax(1) = subplot(5,1,1);
plot(t, L.B, 'LineWidth', 1.2); hold on; yline(0, 'k:');
ylabel('B [m/s]'); title('Barrier value'); grid on;
ax(2) = subplot(5,1,2);
plot(t, L.dzeta, t, L.v_min, '--', 'LineWidth', 1.2);
ylabel('[m/s]'); legend('\zeta dot', 'v_{min}(\zeta)', 'Location', 'best'); grid on;
ax(3) = subplot(5,1,3);
plot(t, L.f_n, t, -L.F_ext_cbf(:,3), t, -L.F_ext_est(:,3), ':', t, L.wrench_raw(:,3), '-.', 'LineWidth', 1.1);
ylabel('[N]'); grid on;
legend('f_n (CBF)', '-F_{ext,z} fed to CBF', '-F_{ext,z} estimate', 'raw O\_F\_ext\_hat\_K z', 'Location', 'best');
title('Braking force vs. push along the normal');
ax(4) = subplot(5,1,4);
plot(t, L.tau_cbf, 'LineWidth', 1.0); ylabel('\tau_{cbf} [Nm]'); grid on;
ax(5) = subplot(5,1,5);
plot(t, L.zeta, 'LineWidth', 1.2); hold on; yline(0, 'k:');
ylabel('\zeta [m]'); xlabel('t [s]'); grid on;
yyaxis right; plot(t, L.slack, 'LineWidth', 1.0); ylabel('slack');
linkaxes(ax, 'x');
end
