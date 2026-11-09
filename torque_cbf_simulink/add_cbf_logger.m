function add_cbf_logger(model)
% ADD_CBF_LOGGER Adds a 'CBF_Logger' subsystem to '<model>/task space controller'.
%
% Every sample it records one row (To Workspace, 'Array' format, variable 'cbf_log'):
%
%   col  1        t            time [s]
%   cols 2-9      telemetry    [is_active, zeta, dzeta, v_min, B, f_n, slack, qp_cost]
%   cols 10-15    F_ext_cbf    wrench actually fed to the CBF S-function (input 11)
%   col  16       use_fext     F_ext compensation flag fed to the CBF (input 12)
%   cols 17-22    F_ext_est    filtered/processed estimate (Goto tag 'F_ext')
%   cols 23-28    wrench_raw   raw robot wrench O_F_ext_hat_K (Goto tag 'wrench_ext')
%   cols 29-35    tau_cbf      CBF joint torque
%   cols 36-42    tau_stiff    lateral spring-damper torque (zeros if not present)
%   cols 43-49    tau_null     nullspace torque
%
% On the robot the Franka target writes To Workspace data to '<model>.mat' when the
% application stops (MAT-file logging, variable 'rt_cbf_log'). Load and plot it with
% load_cbf_log.m, which uses the same column layout.
%
% Sample decimation: workspace variable cbf_log_decimation (see init_torque_cbf.m).
% Close the model in other sessions before running; the script saves the model.
% Safe to run more than once (the logger is rebuilt).

if nargin < 1
    model = 'Controller_trorque_cbf_22Franka';
end
load_system(model);
sub  = [model '/task space controller'];
sfun = 'S-Function Torque CBF';
lname = 'CBF_Logger';
lpath = [sub '/' lname];

% Input definition: {port label, width, source}
%   source = {'branch', block, port}  -> new branch from an existing output port
%          = {'from',  tag}           -> From block on that Goto tag
%          = {'zeros', n}             -> constant zeros (signal missing in model)
sfun_ph = get_param([sub '/' sfun], 'PortHandles');
src11 = line_source(sfun_ph.Inport(11));
src12 = line_source(sfun_ph.Inport(12));
has_stiff = ~isempty(find_system(sub, 'SearchDepth', 1, 'BlockType', 'Goto', 'GotoTag', 'tau_stiff'));
inputs = {
    'telemetry',  8, {'branch', sfun, 2}
    'F_ext_cbf',  6, src11
    'use_fext',   1, src12
    'F_ext_est',  6, {'from', 'F_ext'}
    'wrench_raw', 6, {'from', 'wrench_ext'}
    'tau_cbf',    7, {'from', 'tau_cbf'}
    'tau_stiff',  7, {'from', 'tau_stiff'}
    'tau_null',   7, {'from', 'tau_null'}
};
if ~has_stiff
    inputs{7,3} = {'zeros', 7};
end
nin = size(inputs, 1);

% 1. (Re)build the logger subsystem
if ~isempty(find_system(sub, 'SearchDepth', 1, 'Name', lname))
    lh = get_param(lpath, 'LineHandles');
    for k = 1:numel(lh.Inport)
        if lh.Inport(k) ~= -1, delete_line(lh.Inport(k)); end
    end
    delete_block(lpath);
end
x0 = 1800; y0 = 2400;
add_block('simulink/Ports & Subsystems/Subsystem', lpath, ...
          'Position', [x0, y0, x0+160, y0+30*nin]);
delete_line(lpath, 'In1/1', 'Out1/1');
delete_block([lpath '/In1']);
delete_block([lpath '/Out1']);

% Inside: Inports -> Reshape (1-D) -> Mux (with time first) -> To Workspace
add_block('simulink/Sources/Digital Clock', [lpath '/Clock'], ...
          'SampleTime', 'cbf_dt', 'Position', [200, 20, 240, 40]);
add_block('simulink/Signal Routing/Mux', [lpath '/Mux'], ...
          'Inputs', num2str(nin + 1), 'Position', [420, 10, 425, 60 + 40*nin]);
add_line(lpath, 'Clock/1', 'Mux/1');
for k = 1:nin
    nm = inputs{k,1};
    add_block('simulink/Sources/In1', [lpath '/' nm], ...
              'Position', [40, 55 + 40*k, 70, 70 + 40*k]);
    add_block('simulink/Math Operations/Reshape', [lpath '/Reshape_' nm], ...
              'OutputDimensionality', '1-D array', 'Position', [200, 52 + 40*k, 240, 72 + 40*k]);
    add_line(lpath, [nm '/1'], ['Reshape_' nm '/1']);
    add_line(lpath, ['Reshape_' nm '/1'], ['Mux/' num2str(k + 1)]);
end
add_block('simulink/Sinks/To Workspace', [lpath '/To Workspace cbf_log'], ...
          'VariableName', 'cbf_log', 'SaveFormat', 'Array', 'MaxDataPoints', 'inf', ...
          'Decimation', 'cbf_log_decimation', 'SampleTime', '-1', ...
          'Position', [500, 20 + 20*nin, 600, 50 + 20*nin]);
add_line(lpath, 'Mux/1', 'To Workspace cbf_log/1');

% 2. Connect the inputs in the parent subsystem
for k = 1:nin
    src = inputs{k,3};
    dst = [lname '/' num2str(k)];
    switch src{1}
        case 'branch'
            add_line(sub, [src{2} '/' num2str(src{3})], dst, 'autorouting', 'on');
        case 'from'
            fname = ['From_log_' inputs{k,1}];
            if isempty(find_system(sub, 'SearchDepth', 1, 'Name', fname))
                add_block('simulink/Signal Routing/From', [sub '/' fname], 'GotoTag', src{2}, ...
                          'Position', [x0-150, y0+30*(k-1)+5, x0-70, y0+30*(k-1)+25]);
            else
                old = get_param([sub '/' fname], 'LineHandles');
                if old.Outport ~= -1, delete_line(old.Outport); end
            end
            add_line(sub, [fname '/1'], dst, 'autorouting', 'on');
        case 'zeros'
            cname = ['Const_log_' inputs{k,1}];
            if isempty(find_system(sub, 'SearchDepth', 1, 'Name', cname))
                add_block('simulink/Sources/Constant', [sub '/' cname], ...
                          'Value', sprintf('zeros(%d,1)', src{2}), ...
                          'Position', [x0-150, y0+30*(k-1)+5, x0-70, y0+30*(k-1)+25]);
            else
                old = get_param([sub '/' cname], 'LineHandles');
                if old.Outport ~= -1, delete_line(old.Outport); end
            end
            add_line(sub, [cname '/1'], dst, 'autorouting', 'on');
    end
end

save_system(model);
fprintf('CBF_Logger added to %s (variable cbf_log, %d columns).\n', model, 1 + sum([inputs{:,2}]));
if ~has_stiff
    fprintf('  note: no tau_stiff Goto found, logging zeros in its place.\n');
end
end

% -------------------------------------------------------------------------
function src = line_source(inport_handle)
% Returns {'branch', block, port} for the signal currently feeding an input port
lh = get_param(inport_handle, 'Line');
if lh == -1
    error('add_cbf_logger:unconnected', 'S-function input is not connected.');
end
blk = get_param(get_param(lh, 'SrcBlockHandle'), 'Name');
port = get_param(get_param(lh, 'SrcPortHandle'), 'PortNumber');
src = {'branch', blk, port};
end
