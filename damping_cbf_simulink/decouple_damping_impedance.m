function decouple_damping_impedance(model)
% DECOUPLE_DAMPING_IMPEDANCE Makes the Cartesian impedance of the damping-CBF model act
% only off the plane normal.
%
% In 'impedance controller1' the damping matrix Dd = D_lat + d_cbf*n*n' contains the lateral
% impedance damping D_lat and the CBF damping d_cbf along the normal n (MatrixMultiply3).
% The impedance force is split into
%   F_lat = -(Dd - (n'*Dd*n)*n*n')*V_tilde - Fk          (spring + lateral damping)
%   F_cbf = -(n'*Dd*n) * n * (n'*V_tilde)                 (CBF damping, unchanged)
% and, with cbf_stiff_decouple = 1, F_lat is dynamically decoupled from the normal:
%   F_lat <- F_lat - n*(n'*Lambda^-1*F_lat)/(n'*Lambda^-1*n)
% so the spring/lateral damping produce ZERO acceleration along n (through the arm inertia a
% lateral force or a moment otherwise also accelerates the end-effector along the normal).
% Coriolis/feed-forward terms of the impedance law are unchanged.
%
% 'Compute_F_nominal' (nominal force seen by the damping CBF, a_drift = n'*F_nom/rho) then
% gets its normal component removed, which is exactly the normal effect of the decoupled
% impedance (zero). Before, the CBF ignored the inertial coupling of these forces.
%
% With cbf_stiff_decouple = 0 both blocks give exactly the original result.
% The plane normal constant feeding Goto 'n_hp' is set from plane_normal (CBF plane).
% Requires cbf_stiff_decouple and plane_normal in the workspace (init_damping_cbf.m).
% Close the model in other sessions before running; the script saves the model.
% Safe to run more than once.

if nargin < 1
    model = 'Controller_damping_cbf_22Franka';
end
load_system(model);
sub = [model '/task space controller'];
imp = [sub '/impedance controller1'];
fnom = [sub '/Compute_F_nominal'];

% 1. Impedance law
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', imp);
chart.Script = impedance_code();
add_or_get_const(sub, 'Const_StiffDecouple_imp', 'cbf_stiff_decouple', block_left_of(imp, 15));
safe_add_line(sub, 'Const_StiffDecouple_imp/1', 'impedance controller1/15');

% 2. Nominal force for the damping CBF
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', fnom);
chart.Script = fnom_code();
add_or_get_from(sub, 'From_nhp_fnom', 'n_hp', block_left_of(fnom, 4));
add_or_get_const(sub, 'Const_StiffDecouple_fnom', 'cbf_stiff_decouple', block_left_of(fnom, 5));
safe_add_line(sub, 'From_nhp_fnom/1', 'Compute_F_nominal/4');
safe_add_line(sub, 'Const_StiffDecouple_fnom/1', 'Compute_F_nominal/5');

% 3. Plane normal from plane_normal
g = find_system(sub, 'SearchDepth', 1, 'BlockType', 'Goto', 'GotoTag', 'n_hp');
lh = get_param(g{1}, 'LineHandles');
src = get_param(lh.Inport(1), 'SrcBlockHandle');
if strcmp(get_param(src, 'BlockType'), 'Constant')
    set_param(src, 'Value', '[plane_normal(:)./norm(plane_normal); 0; 0; 0]');
end

save_system(model);
fprintf('Impedance of %s decoupled from the plane normal (switch: cbf_stiff_decouple).\n', model);
end

% -------------------------------------------------------------------------
function code = impedance_code()
code = strjoin({ ...
'function F_control  = Cartesian_impedance_control(q_dot, V_tilde, V_ref_dot, massM, lambdaM, C, Jg,...'
'    Jg_dot, JgRightPseu, Fk, Dd, H_I_D, n_hp, d_add, decouple)'
''
'% Dd = diag(Dd);'
'R_I_D = H_I_D(1:3,1:3);'
'V_ref_dot = blkdiag(R_I_D,R_I_D)*V_ref_dot;'
'v_c = dot(n_hp, V_tilde);'
''
'massMInv = massM\eye(7);'
'massMInv = (massMInv + transpose(massMInv))./2;'
'mu = lambdaM*(Jg*massMInv*C - Jg_dot);'
''
'Lambda_inv = Jg*massMInv*transpose(Jg);'
'Lambda_inv = (Lambda_inv + transpose(Lambda_inv))./2;'
''
'% Split Dd = D_lat + d_n*n*n'': D_lat = impedance damping, d_n = CBF damping along n'
'n6 = n_hp(:)./norm(n_hp);'
'd_n = transpose(n6)*Dd*n6;'
'D_lat = Dd - d_n.*(n6*transpose(n6));'
''
'% Spring + lateral damping'
'F_lat = -D_lat*V_tilde - Fk;'
'if decouple > 0.5'
'    % remove the acceleration F_lat would cause along the plane normal (Lambda^-1 coupling)'
'    F_lat = F_lat - n6.*((transpose(n6)*Lambda_inv*F_lat)/(transpose(n6)*Lambda_inv*n6));'
'end'
''
'% CBF damping along the normal (unchanged)'
'F_cbf = -d_n.*n6.*(transpose(n6)*V_tilde);'
''
'% Variante 1 (decouple = 0: identical to -Dd*V_tilde - Fk)'
'F_c = mu*q_dot + lambdaM*V_ref_dot - mu*JgRightPseu*V_tilde + F_lat + F_cbf;'
''
'm_c = (transpose(n_hp)*Lambda_inv*n_hp)\eye(1);'
'C_c = (m_c^2) .*(transpose(n_hp) *Lambda_inv * mu*JgRightPseu * Lambda_inv * n_hp);'
''
'% Variante 2'
'% F_c  =  mu*q_dot + lambdaM*V_ref_dot + (m_c.*(n_hp*transpose(n_hp))*Lambda_inv - eye(6))*(mu*JgRightPseu*V_tilde + Dd*V_tilde + Fk) - (C_c+d_add).*(n_hp*transpose(n_hp)*V_tilde);'
'F_c2  =  n_hp*( m_c.*(transpose(n_hp)*Lambda_inv)*(mu*JgRightPseu*V_tilde + Dd*V_tilde + Fk)) + n_hp.*(-C_c*v_c - d_add*v_c); %#ok<NASGU>'
''
'% F_control = F_c + F_c2;'
'F_control = F_c ;'
''
'end'
}, newline);
end

function code = fnom_code()
code = strjoin({ ...
'function F_nominal = fcn(Fk, D0, V_tilde, n_hp, decouple)'
'% Nominal impedance wrench pulling toward reference: -Fk - D0 * V_tilde'
'F_nominal = -Fk - D0 * V_tilde;'
'if decouple > 0.5'
'    % The decoupled impedance produces no acceleration along the plane normal, so the CBF'
'    % (a_drift = n''*F_nominal/rho) must see no nominal normal force.'
'    n6 = n_hp(:)./norm(n_hp);'
'    F_nominal = F_nominal - n6.*(transpose(n6)*F_nominal);'
'end'
'end'
}, newline);
end

function pos = block_left_of(blk, port)
% Position for a source block left of input 'port' of 'blk'
ph = get_param(blk, 'PortHandles');
p = get_param(ph.Inport(port), 'Position');
pos = [p(1)-160, p(2)-10, p(1)-80, p(2)+10];
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
% Connect src -> dst, replacing any line already feeding dst or leaving src
dst_parts = strsplit(dst, '/');
ph = get_param([sys '/' dst_parts{1}], 'PortHandles');
port_num = str2double(dst_parts{2});
if port_num <= numel(ph.Inport)
    lh = get_param(ph.Inport(port_num), 'Line');
    if lh ~= -1, delete_line(lh); end
end
src_parts = strsplit(src, '/');
sph = get_param([sys '/' src_parts{1}], 'PortHandles');
lh = get_param(sph.Outport(str2double(src_parts{2})), 'Line');
if lh ~= -1, delete_line(lh); end
add_line(sys, src, dst, 'autorouting', 'on');
end
