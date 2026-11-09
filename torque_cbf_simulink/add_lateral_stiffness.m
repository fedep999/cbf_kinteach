function add_lateral_stiffness(model)
% ADD_LATERAL_STIFFNESS Adds a Cartesian spring-damper (x/y + orientation) to the torque-CBF model.
%
%   tau_stiff = -Jg' * (Fk + Fd)
%   Fk = [KdTrans .* (x - x_ref); rotational spring(KdRot)]      (spring towards H_ref)
%   Fd = D .* (Jg*q_dot),  D_i = 2*zeta_i*sqrt(K_i * Lambda_ii)  (damping, ratio zeta_i)
%
% Same spring as 'Stiffnessforces/MATLAB Function3' of the damping model (reference pose
% H_ref = H_home), plus damping scaled with the Cartesian mass matrix Lambda so the damping
% ratio is the same in every configuration. No Coriolis/feed-forward terms.
% With KdTrans = [Kx; Ky; 0] and ZetaTrans = [zx; zy; 0] the end-effector is held on x/y
% (and in orientation) and no spring/damper force acts along z.
%
% Dynamic decoupling from the plane normal (cbf_stiff_decouple = 1):
%   F <- F - n6 * (n6'*Lambda^-1*F) / (n6'*Lambda^-1*n6),   n6 = [n_plane; 0; 0; 0]
% Through the arm inertia a lateral force or a moment at the flange also accelerates the
% end-effector along the normal (Lambda^-1 is not diagonal). The correction adds exactly
% the normal force that cancels it, so the spring-damper produces ZERO acceleration along
% the plane normal and the CBF is the only controller acting in that direction.
% cbf_stiff_decouple = 0 gives the plain spring-damper.
% The normal n_hp is set from plane_normal (same plane as the CBF).
%
% Wiring performed (inside '<model>/task space controller'):
%   H_IT, H_ref, Jg_mat, cbf_Kd_trans, cbf_Kd_rot, q_dot, lambda_mat,
%   cbf_zeta_trans, cbf_zeta_rot, n_hp, cbf_stiff_decouple -> Lateral_Stiffness -> Goto 'tau_stiff'
%   Sum (commanded torque): tau_null + 0 + tau_cbf + tau_stiff
%   S-Function Torque CBF input 9 (tau_nom): tau_null + tau_stiff
%     (tau_stiff must be part of tau_nom: through the arm inertia the lateral spring also
%      accelerates the end-effector along z, and the CBF accounts for it only via tau_nom;
%      it also enters the total-torque limit)
%
% Requires cbf_Kd_trans, cbf_Kd_rot, cbf_zeta_trans, cbf_zeta_rot, cbf_stiff_decouple and
% plane_normal in the workspace (see init_torque_cbf.m).
% Close the model in other sessions before running; the script saves the model.
% Safe to run more than once.

if nargin < 1
    model = 'Controller_trorque_cbf_22Franka';
end
load_system(model);
sub = [model '/task space controller'];
sfun = 'S-Function Torque CBF';

% Free area of the diagram for the new blocks
x0 = 900; y0 = 2400;

% 1. MATLAB Function block computing tau_stiff
fname = 'Lateral_Stiffness';
fpath = [sub '/' fname];
if isempty(find_system(sub, 'SearchDepth', 1, 'Name', fname))
    add_block('simulink/User-Defined Functions/MATLAB Function', fpath, ...
              'Position', [x0+200, y0, x0+360, y0+140]);
end
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', fpath);
chart.Script = stiffness_code();

% 2. Inputs
add_or_get_from(sub, 'From_HIT_stiff',  'H_IT',   [x0, y0+5,   x0+80, y0+25]);
add_or_get_from(sub, 'From_Href_stiff', 'H_ref',  [x0, y0+35,  x0+80, y0+55]);
add_or_get_from(sub, 'From_Jg_stiff',   'Jg_mat', [x0, y0+65,  x0+80, y0+85]);
add_or_get_const(sub, 'Const_KdTrans', 'cbf_Kd_trans', [x0, y0+95,  x0+80, y0+115]);
add_or_get_const(sub, 'Const_KdRot',   'cbf_Kd_rot',   [x0, y0+125, x0+80, y0+145]);
add_or_get_from(sub, 'From_qdot_stiff',   'q_dot',      [x0, y0+155, x0+80, y0+175]);
add_or_get_from(sub, 'From_lambda_stiff', 'lambda_mat', [x0, y0+185, x0+80, y0+205]);
add_or_get_const(sub, 'Const_ZetaTrans', 'cbf_zeta_trans', [x0, y0+215, x0+80, y0+235]);
add_or_get_const(sub, 'Const_ZetaRot',   'cbf_zeta_rot',   [x0, y0+245, x0+80, y0+265]);
add_or_get_from(sub, 'From_nhp_stiff', 'n_hp', [x0, y0+275, x0+80, y0+295]);
add_or_get_const(sub, 'Const_StiffDecouple', 'cbf_stiff_decouple', [x0, y0+305, x0+80, y0+325]);
set_param(fpath, 'Position', [x0+200, y0, x0+360, y0+330]);
safe_add_line(sub, 'From_HIT_stiff/1',    [fname '/1']);
safe_add_line(sub, 'From_Href_stiff/1',   [fname '/2']);
safe_add_line(sub, 'From_Jg_stiff/1',     [fname '/3']);
safe_add_line(sub, 'Const_KdTrans/1',     [fname '/4']);
safe_add_line(sub, 'Const_KdRot/1',       [fname '/5']);
safe_add_line(sub, 'From_qdot_stiff/1',   [fname '/6']);
safe_add_line(sub, 'From_lambda_stiff/1', [fname '/7']);
safe_add_line(sub, 'Const_ZetaTrans/1',   [fname '/8']);
safe_add_line(sub, 'Const_ZetaRot/1',     [fname '/9']);
safe_add_line(sub, 'From_nhp_stiff/1',    [fname '/10']);
safe_add_line(sub, 'Const_StiffDecouple/1', [fname '/11']);

% Plane normal used by the controller blocks (Goto 'n_hp'): same plane as the CBF
set_plane_normal_constant(sub);

% 3. Goto 'tau_stiff'
if isempty(find_system(sub, 'SearchDepth', 1, 'Name', 'Goto_tau_stiff'))
    add_block('simulink/Signal Routing/Goto', [sub '/Goto_tau_stiff'], ...
              'GotoTag', 'tau_stiff', 'Position', [x0+420, y0+60, x0+500, y0+80]);
end
safe_add_line(sub, [fname '/1'], 'Goto_tau_stiff/1');

% 4. Commanded torque: Sum = tau_null + 0 + tau_cbf + tau_stiff
set_param([sub '/Sum'], 'Inputs', '++++');
add_or_get_from(sub, 'From_tau_stiff_cmd', 'tau_stiff', [x0+420, y0+120, x0+500, y0+140]);
safe_add_line(sub, 'From_tau_stiff_cmd/1', 'Sum/4');

% 5. CBF nominal torque (S-function input 9): tau_null + tau_stiff
snom = 'Sum_tau_nom_cbf';
if isempty(find_system(sub, 'SearchDepth', 1, 'Name', snom))
    add_block('simulink/Math Operations/Sum', [sub '/' snom], ...
              'Inputs', '++', 'Position', [x0+620, y0+160, x0+650, y0+200]);
end
add_or_get_from(sub, 'From_taunull_nom', 'tau_null', [x0+520, y0+160, x0+600, y0+180]);
add_or_get_from(sub, 'From_tau_stiff_nom', 'tau_stiff', [x0+520, y0+190, x0+600, y0+210]);
safe_add_line(sub, [snom '/1'], [sfun '/9']);              % replaces tau_null -> input 9
safe_add_line(sub, 'From_taunull_nom/1', [snom '/1']);
safe_add_line(sub, 'From_tau_stiff_nom/1', [snom '/2']);

save_system(model);
fprintf('Lateral spring-damper added to %s (tau_stiff = -Jg''*(Fk + Fd)).\n', model);
end

% -------------------------------------------------------------------------
function code = stiffness_code()
code = sprintf([ ...
'function tau_stiff = fcn(H_I_t, H_I_D, Jg, KdTrans, KdRot, q_dot, Lambda, ZetaTrans, ZetaRot, n_hp, decouple)\n' ...
'%%%% Cartesian spring-damper torque (no Coriolis / feed-forward terms).\n' ...
'%%%% Spring as Stiffnessforces/MATLAB Function3 of the damping model:\n' ...
'%%%%   Fk = [KdTrans*(x - x_D); 2*R_D*(eta*I + skew(eps))*KdRot*eps]\n' ...
'%%%% Damping with constant damping ratio (Lambda = Cartesian mass matrix):\n' ...
'%%%%   Fd = D .* (Jg*q_dot),  D_i = 2*zeta_i*sqrt(K_i*Lambda_ii)\n' ...
'%%%% Decoupling from the plane normal n6 (decouple = 1):\n' ...
'%%%%   F = F - n6*(n6''*Lambda^-1*F)/(n6''*Lambda^-1*n6)  -> zero acceleration along n6\n' ...
'%%%%   tau = -Jg''*F\n' ...
'lam = diag(Lambda);\n' ...
'Dtrans = 2.*ZetaTrans(:).*sqrt(KdTrans(:).*max(lam(1:3), 0));\n' ...
'Drot   = 2.*ZetaRot(:).*sqrt(KdRot(:).*max(lam(4:6), 0));\n' ...
'V = Jg*q_dot;\n' ...
'Fd = [Dtrans.*V(1:3); Drot.*V(4:6)];\n' ...
'\n' ...
'KdTrans = diag(KdTrans);\n' ...
'KdRot = diag(KdRot);\n' ...
'\n' ...
'R_D_t = (H_I_D(1:3,1:3))''*H_I_t(1:3,1:3);\n' ...
'\n' ...
'eta_D_t = 0.5*sqrt(abs(1 + R_D_t(1,1) + R_D_t(2,2) + R_D_t(3,3)));\n' ...
'epsi_D_t = 0.5.*[MySign(R_D_t(3,2) - R_D_t(2,3))*realsqrt(abs( R_D_t(1,1)-R_D_t(2,2)-R_D_t(3,3)+1));\n' ...
'                 MySign(R_D_t(1,3) - R_D_t(3,1))*realsqrt(abs(-R_D_t(1,1)+R_D_t(2,2)-R_D_t(3,3)+1));\n' ...
'                 MySign(R_D_t(2,1) - R_D_t(1,2))*realsqrt(abs(-R_D_t(1,1)-R_D_t(2,2)+R_D_t(3,3)+1))];\n' ...
'\n' ...
'epsi_D_t_skew = [0,           -epsi_D_t(3),  epsi_D_t(2);\n' ...
'                 epsi_D_t(3),   0,           -epsi_D_t(1);\n' ...
'                -epsi_D_t(2),   epsi_D_t(1),  0];\n' ...
'\n' ...
'x_tilde = H_I_t(1:3,4) - H_I_D(1:3,4);\n' ...
'\n' ...
'Fktrans = KdTrans*x_tilde;\n' ...
'Fkrot = 2.*H_I_D(1:3,1:3)*(eta_D_t.*eye(3) + epsi_D_t_skew)*KdRot*epsi_D_t;\n' ...
'Fk = [Fktrans; Fkrot];\n' ...
'F = Fk + Fd;\n' ...
'\n' ...
'if decouple > 0.5\n' ...
'    n6 = n_hp(:);\n' ...
'    n6 = n6 ./ norm(n6);\n' ...
'    Lambda_inv = Lambda \\ eye(6);\n' ...
'    Lambda_inv = 0.5.*(Lambda_inv + transpose(Lambda_inv));\n' ...
'    a_n = transpose(n6)*Lambda_inv*F;          %%%% normal acceleration caused by F\n' ...
'    c_n = transpose(n6)*Lambda_inv*n6;         %%%% = 1/rho along the normal\n' ...
'    F = F - n6.*(a_n/c_n);\n' ...
'end\n' ...
'\n' ...
'tau_stiff = -transpose(Jg)*F;\n' ...
'end\n' ...
'\n' ...
'function x = MySign(val)\n' ...
'%%%% sign function with MySign(0) = 1 (Siciliano 2010, eq. 2.35)\n' ...
'x = 1;\n' ...
'if (val < 0)\n' ...
'    x = -1;\n' ...
'end\n' ...
'end\n']);
end

function set_plane_normal_constant(sub)
% The Goto 'n_hp' is fed by a Constant (hardcoded [0;0;1;0;0;0] originally): drive it from
% plane_normal so every block using n_hp refers to the same plane as the CBF.
g = find_system(sub, 'SearchDepth', 1, 'BlockType', 'Goto', 'GotoTag', 'n_hp');
if isempty(g)
    warning('add_lateral_stiffness:nhp', 'No Goto ''n_hp'' found: plane normal constant not updated.');
    return;
end
lh = get_param(g{1}, 'LineHandles');
src = get_param(lh.Inport(1), 'SrcBlockHandle');
if strcmp(get_param(src, 'BlockType'), 'Constant')
    set_param(src, 'Value', '[plane_normal(:)./norm(plane_normal); 0; 0; 0]');
else
    warning('add_lateral_stiffness:nhp', 'Goto ''n_hp'' is not fed by a Constant: left unchanged.');
end
end

function add_or_get_from(subsys, name, tag, pos)
if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', name))
    add_block('simulink/Signal Routing/From', [subsys '/' name], 'GotoTag', tag, 'Position', pos);
end
end

function add_or_get_const(subsys, name, value, pos)
if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', name))
    add_block('simulink/Sources/Constant', [subsys '/' name], 'Value', value, 'Position', pos);
else
    set_param([subsys '/' name], 'Value', value);
end
end

function safe_add_line(sys, src, dst)
% Connect src -> dst, replacing any line already feeding dst
dst_parts = strsplit(dst, '/');
ph = get_param([sys '/' dst_parts{1}], 'PortHandles');
port_num = str2double(dst_parts{2});
if port_num <= numel(ph.Inport)
    lh = get_param(ph.Inport(port_num), 'Line');
    if lh ~= -1
        delete_line(lh);
    end
end
% A From block output may still carry a stale line (e.g. after re-wiring): remove it
src_parts = strsplit(src, '/');
sph = get_param([sys '/' src_parts{1}], 'PortHandles');
src_port = str2double(src_parts{2});
if strcmp(get_param([sys '/' src_parts{1}], 'BlockType'), 'From') && src_port <= numel(sph.Outport)
    lh = get_param(sph.Outport(src_port), 'Line');
    if lh ~= -1
        delete_line(lh);
    end
end
add_line(sys, src, dst, 'autorouting', 'on');
end
