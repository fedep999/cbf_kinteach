% Script to configure Controller_damping_cbf.slx
function setup_damping_model()
    model = 'Controller_damping_cbf';
    load_system(model);
    
    subsys = [model '/task space controller'];
    
    % Remove legacy empirical damping blocks if present
    legacy_blks = {'MATLAB Function5', 'MATLAB Function6', 'Subsystem5', ...
                   'KindOfDamping1', 'd_const1', 'v_max1', 'dMAX1', 'exponent1', 'Constant9', ...
                   'Memory2', 'From60', 'From39', 'From58', 'From59', ...
                   'Goto27', 'Goto28', 'Goto29', 'Goto30', ...
                   'From62', 'From63', 'From64', 'From65', 'From67', ...
                   'Constant_a_drift', 'From_vc_cbf'};
    for b = 1:length(legacy_blks)
        bp = [subsys '/' legacy_blks{b}];
        if ~isempty(find_system(subsys, 'SearchDepth', 1, 'IncludeCommented', 'on', 'Name', legacy_blks{b}))
            lh = get_param(bp, 'LineHandles');
            if isfield(lh, 'Inport'), for p = 1:length(lh.Inport), if lh.Inport(p) ~= -1, delete_line(lh.Inport(p)); end; end; end
            if isfield(lh, 'Outport'), for p = 1:length(lh.Outport), if lh.Outport(p) ~= -1, delete_line(lh.Outport(p)); end; end; end
            delete_block(bp);
        end
    end
    
    % Ensure Goto26 (tag: damp) line is clean
    goto26_line = get_param([subsys '/Goto26'], 'LineHandles');
    if goto26_line.Inport ~= -1
        delete_line(goto26_line.Inport);
    end
    
    % S-Function block for Damping CBF
    sfun_name = 'S-Function Damping CBF';
    sfun_path = [subsys '/' sfun_name];
    if ~isempty(find_system(subsys, 'SearchDepth', 1, 'Name', sfun_name))
        lh = get_param(sfun_path, 'LineHandles');
        if isfield(lh, 'Inport'), for p = 1:length(lh.Inport), if lh.Inport(p) ~= -1, delete_line(lh.Inport(p)); end; end; end
        if isfield(lh, 'Outport'), for p = 1:length(lh.Outport), if lh.Outport(p) ~= -1, delete_line(lh.Outport(p)); end; end; end
        delete_block(sfun_path);
    end
    add_block('simulink/User-Defined Functions/S-Function', sfun_path, ...
              'FunctionName', 'sfun_damping_cbf', ...
              'Position', [650, 3100, 850, 3300]);
    
    % 1. Helper: Compute true ground velocity vc_ground = n_hp' * (Jg * q_dot)
    vc_helper_name = 'Compute_vc_ground';
    vc_helper_path = [subsys '/' vc_helper_name];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', vc_helper_name))
        add_block('simulink/User-Defined Functions/MATLAB Function', vc_helper_path, ...
                  'Position', [480, 3050, 580, 3110]);
        sf = sfroot;
        chart = sf.find('Path', vc_helper_path, '-isa', 'Stateflow.EMChart');
        if ~isempty(chart)
            chart.Script = sprintf(['function vc = fcn(n_hp, Jg, q_dot)\n' ...
                                    '%% Ground velocity along hyperplane normal: n^T * J_trans * q_dot\n' ...
                                    'vc = dot(n_hp(1:3), Jg(1:3, :) * q_dot);\n' ...
                                    'end\n']);
        end
    end
    
    % From blocks for vc_helper
    add_or_get_from(subsys, 'From_nhp_damp', 'n_hp', [380, 3050, 440, 3070]);
    add_or_get_from(subsys, 'From_Jg_damp', 'Jg_mat', [380, 3075, 440, 3095]);
    add_or_get_from(subsys, 'From_qdot_damp', 'q_dot', [380, 3100, 440, 3120]);
    safe_add_line(subsys, 'From_nhp_damp/1', [vc_helper_name '/1']);
    safe_add_line(subsys, 'From_Jg_damp/1', [vc_helper_name '/2']);
    safe_add_line(subsys, 'From_qdot_damp/1', [vc_helper_name '/3']);
    
    % 2. Helper: Compute nominal impedance wrench F_nominal = -Fk - D0 * V_tilde
    fnom_helper_name = 'Compute_F_nominal';
    fnom_helper_path = [subsys '/' fnom_helper_name];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', fnom_helper_name))
        add_block('simulink/User-Defined Functions/MATLAB Function', fnom_helper_path, ...
                  'Position', [480, 3130, 580, 3190]);
        sf = sfroot;
        chart = sf.find('Path', fnom_helper_path, '-isa', 'Stateflow.EMChart');
        if ~isempty(chart)
            chart.Script = sprintf(['function F_nominal = fcn(Fk, D0, V_tilde)\n' ...
                                    '%% Nominal impedance wrench pulling toward reference: -Fk - D0 * V_tilde\n' ...
                                    'F_nominal = -Fk - D0 * V_tilde;\n' ...
                                    'end\n']);
        end
    end
    
    % From block for V_tilde
    add_or_get_from(subsys, 'From_Vtilde_damp', 'V_tilde', [380, 3175, 440, 3195]);
    % Connect Stiffnessforces/1 (Fk) and Create Diagonal Matrix1/1 (D0) to fnom_helper
    safe_add_line(subsys, 'Stiffnessforces/1', [fnom_helper_name '/1']);
    safe_add_line(subsys, 'Create Diagonal Matrix1/1', [fnom_helper_name '/2']);
    safe_add_line(subsys, 'From_Vtilde_damp/1', [fnom_helper_name '/3']);
    
    % 3. Inputs to S-Function Damping CBF:
    % Inport 1: zeta (x_c)
    add_or_get_from(subsys, 'From_xc_cbf', 'x_c', [480, 3015, 540, 3035]);
    safe_add_line(subsys, 'From_xc_cbf/1', [sfun_name '/1']);
    
    % Inport 2: dzeta (vc from vc_helper)
    safe_add_line(subsys, [vc_helper_name '/1'], [sfun_name '/2']);
    
    % Inport 3: rho (mass_c)
    add_or_get_from(subsys, 'From_mc_cbf', 'mass_c', [480, 3105, 540, 3125]);
    safe_add_line(subsys, 'From_mc_cbf/1', [sfun_name '/3']);
    
    % Inport 4: F_nominal (6x1 from fnom_helper)
    safe_add_line(subsys, [fnom_helper_name '/1'], [sfun_name '/4']);
    
    % Inport 5: F_ext (6x1 Constant zeros)
    fext_path = [subsys '/Constant_Fext_damp'];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', 'Constant_Fext_damp'))
        add_block('simulink/Sources/Constant', fext_path, ...
                  'Value', 'zeros(6,1)', 'Position', [480, 3205, 540, 3225]);
    end
    safe_add_line(subsys, 'Constant_Fext_damp/1', [sfun_name '/5']);
    
    % Inport 6: use_fext (1x1 Constant 0 / false)
    usefext_path = [subsys '/Constant_use_fext_damp'];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', 'Constant_use_fext_damp'))
        add_block('simulink/Sources/Constant', usefext_path, ...
                  'Value', '0', 'Position', [480, 3235, 540, 3255]);
    end
    safe_add_line(subsys, 'Constant_use_fext_damp/1', [sfun_name '/6']);
    
    % Inport 7: cbf_params (13x1 Constant cbf_damping_params)
    params_path = [subsys '/Constant_cbf_params'];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', 'Constant_cbf_params'))
        add_block('simulink/Sources/Constant', params_path, ...
                  'Value', 'cbf_damping_params', 'Position', [450, 3265, 570, 3285]);
    end
    safe_add_line(subsys, 'Constant_cbf_params/1', [sfun_name '/7']);
    
    % Connect Outport 1 (d_cbf) to Goto26 (tag: damp)
    safe_add_line(subsys, [sfun_name '/1'], 'Goto26/1');
    
    % Visualizer for Outport 2 (telemetry)
    vis_name = 'CBF_Telemetry_Visualizer';
    vis_path = [subsys '/' vis_name];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', vis_name))
        add_damping_visualizer();
    else
        safe_add_line(subsys, [sfun_name '/2'], [vis_name '/1']);
    end
    
    save_system(model);
    bdclose(model);
    disp('Successfully configured Controller_damping_cbf.slx!');
end

function add_or_get_from(subsys, name, tag, pos)
    p = [subsys '/' name];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', name))
        add_block('simulink/Signal Routing/From', p, ...
                  'GotoTag', tag, 'Position', pos);
    end
end

function safe_add_line(sys, src, dst)
    dst_parts = strsplit(dst, '/');
    dst_blk = [sys '/' dst_parts{1}];
    port_num = str2double(dst_parts{2});
    ph = get_param(dst_blk, 'PortHandles');
    if ~isempty(ph.Inport) && port_num <= length(ph.Inport)
        lh = get_param(ph.Inport(port_num), 'Line');
        if lh ~= -1
            delete_line(lh);
        end
    end
    add_line(sys, src, dst, 'autorouting', 'on');
end
