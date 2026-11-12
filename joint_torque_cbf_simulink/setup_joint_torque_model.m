function setup_joint_torque_model(source_model)
% SETUP_JOINT_TORQUE_MODEL Creates Controller_jtorque_cbf_22Franka.slx from the real-robot torque CBF
% model, replacing the torque CBF S-function by the joint-space torque CBF (sfun_joint_torque_cbf).
%
%   setup_joint_torque_model                     % from ../torque_cbf_simulink/Controller_trorque_cbf_22Franka.slx
%   setup_joint_torque_model('path/to/model.slx')
%
% Changes made in '<model>/task space controller' (everything else, incl. wiring, is kept):
%   - S-Function 'S-Function Torque CBF': FunctionName -> sfun_joint_torque_cbf
%     (same 13 inputs; the new 3rd output 'guarantees' goes to a Scope 'JTorque_Guarantees')
%   - Constant 'Constant_torque_params': Value -> cbf_jtorque_params
% Code generation (Franka target): Custom include/source paths point to joint_torque_cbf_controller.
%
% Run init_joint_torque_cbf.m first (workspace parameters), with the Franka Simulink library on
% the MATLAB path. The source model is not modified. Safe to run again (overwrites the target).

this_dir = fileparts(mfilename('fullpath'));
ws_dir = fileparts(this_dir);
if nargin < 1
    source_model = fullfile(ws_dir, 'torque_cbf_simulink', 'Controller_trorque_cbf_22Franka.slx');
end
target_name = 'Controller_jtorque_cbf_22Franka';
target_file = fullfile(this_dir, [target_name '.slx']);

[~, src_name] = fileparts(source_model);
if bdIsLoaded(src_name)
    error('setup_joint_torque_model:loaded', 'Close %s first (it must not have unsaved changes).', src_name);
end
if bdIsLoaded(target_name)
    close_system(target_name, 0);
end
copyfile(source_model, target_file, 'f');
load_system(target_file);

sub = [target_name '/task space controller'];
sfun = [sub '/S-Function Torque CBF'];

% 1. S-function
set_param(sfun, 'FunctionName', 'sfun_joint_torque_cbf');
set_param(sfun, 'Description', ...
    'Joint-space torque CBF (jtorque_cbf): tau_cbf as QP decision variable, rate-limit aware');

% 2. Parameter vector
set_param([sub '/Constant_torque_params'], 'Value', 'cbf_jtorque_params');

% 3. Third output (guarantees) to a scope
scope = [sub '/JTorque_Guarantees'];
if isempty(find_system(sub, 'SearchDepth', 1, 'Name', 'JTorque_Guarantees'))
    pos = get_param(sfun, 'Position');
    add_block('simulink/Sinks/Scope', scope, ...
              'Position', [pos(3)+120, pos(4)-40, pos(3)+160, pos(4)]);
end
ph = get_param(sfun, 'PortHandles');
if numel(ph.Outport) >= 3 && get_param(ph.Outport(3), 'Line') == -1
    add_line(sub, 'S-Function Torque CBF/3', 'JTorque_Guarantees/1', 'autorouting', 'on');
end

% 4. Code generation: controller sources of the joint-space torque CBF
cs = getActiveConfigSet(target_name);
set_param(cs, 'CustomInclude', strjoin({ ...
    fullfile(ws_dir, 'joint_torque_cbf_controller', 'include'), ...
    fullfile(ws_dir, 'cbf_common', 'include'), ...
    fullfile(fileparts(ws_dir), 'pinocchio_install', 'include')}, newline));
set_param(cs, 'CustomSource', strjoin({ ...
    fullfile(ws_dir, 'joint_torque_cbf_controller', 'src', 'joint_torque_cbf_controller.cpp'), ...
    fullfile(ws_dir, 'cbf_common', 'src', 'profiles.cpp')}, newline));

save_system(target_name, target_file);
close_system(target_name, 0);
fprintf('Created %s (S-function sfun_joint_torque_cbf, parameters cbf_jtorque_params).\n', target_file);
end
