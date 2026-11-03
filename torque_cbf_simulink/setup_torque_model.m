% Script to configure Controller_torque_cbf.slx
function setup_torque_model()
    model = 'Controller_torque_cbf';
    load_system(model);
    
    subsys = [model '/task space controller'];
    
    % Remove legacy empirical damping blocks if present
    legacy_blks = {'MATLAB Function5', 'MATLAB Function6', 'Subsystem5', ...
                   'KindOfDamping1', 'd_const1', 'v_max1', 'dMAX1', 'exponent1', 'Constant9', ...
                   'Memory2', 'From60', 'From39', 'From58', 'From59', ...
                   'Goto27', 'Goto28', 'Goto29', 'Goto30', ...
                   'From62', 'From63', 'From64', 'From65', 'From67'};
    for b = 1:length(legacy_blks)
        bp = [subsys '/' legacy_blks{b}];
        if ~isempty(find_system(subsys, 'SearchDepth', 1, 'IncludeCommented', 'on', 'Name', legacy_blks{b}))
            lh = get_param(bp, 'LineHandles');
            if isfield(lh, 'Inport'), for p = 1:length(lh.Inport), if lh.Inport(p) ~= -1, delete_line(lh.Inport(p)); end; end; end
            if isfield(lh, 'Outport'), for p = 1:length(lh.Outport), if lh.Outport(p) ~= -1, delete_line(lh.Outport(p)); end; end; end
            delete_block(bp);
        end
    end
    
    % Ensure Goto26 (tag: damp) gets zero
    zero_damp_name = 'Constant_zero_damp';
    zero_damp_path = [subsys '/' zero_damp_name];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', zero_damp_name))
        add_block('simulink/Sources/Constant', zero_damp_path, ...
                  'Value', '0', ...
                  'Position', [650, 3100, 680, 3120]);
    end
    safe_add_line(subsys, [zero_damp_name '/1'], 'Goto26/1');
    
    % 1. Tap gravity torque g_vec from Bus Selector2 Outport 4 with a Goto block
    goto_g_name = 'Goto_g_vec';
    goto_g_path = [subsys '/' goto_g_name];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', goto_g_name))
        add_block('simulink/Signal Routing/Goto', goto_g_path, ...
                  'GotoTag', 'g_vec', ...
                  'Position', [-1250, 20, -1190, 40]);
        add_line(subsys, 'Bus Selector2/4', [goto_g_name '/1'], 'autorouting', 'on');
    end
    
    % 2. Tap tau_null with Goto block; terminate tau_task for kinesthetic teaching
    goto_null_name = 'Goto_tau_null';
    goto_null_path = [subsys '/' goto_null_name];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', goto_null_name))
        add_block('simulink/Signal Routing/Goto', goto_null_path, ...
                  'GotoTag', 'tau_null', ...
                  'Position', [2080, -40, 2150, -20]);
        add_line(subsys, 'Manual Switch/1', [goto_null_name '/1'], 'autorouting', 'on');
    end
    
    term_tautask_name = 'Terminator_tau_task';
    term_tautask_path = [subsys '/' term_tautask_name];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', term_tautask_name))
        add_block('simulink/Sinks/Terminator', term_tautask_path, ...
                  'Position', [2025, 285, 2045, 305]);
    end
    safe_add_line(subsys, 'MatrixMultiply1/1', [term_tautask_name '/1']);
    
    % If old Goto_tau_task exists, remove it
    if ~isempty(find_system(subsys, 'SearchDepth', 1, 'Name', 'Goto_tau_task'))
        lh_gt = get_param([subsys '/Goto_tau_task'], 'LineHandles');
        if lh_gt.Inport ~= -1, delete_line(lh_gt.Inport); end
        delete_block([subsys '/Goto_tau_task']);
    end
    
    % 3. Add helper MATLAB Function to extract x_ee and v_ee
    ee_helper_name = 'Extract_EE_Kinematics';
    ee_helper_path = [subsys '/' ee_helper_name];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', ee_helper_name))
        add_block('simulink/User-Defined Functions/MATLAB Function', ee_helper_path, ...
                  'Position', [1200, 1640, 1300, 1720]);
        sf = sfroot;
        chart = sf.find('Path', ee_helper_path, '-isa', 'Stateflow.EMChart');
        if ~isempty(chart)
            chart.Script = sprintf(['function [x_ee, v_ee] = fcn(H, Jg, q_dot)\n' ...
                                    'x_ee = H(1:3, 4);\n' ...
                                    'v_ee = Jg(1:3, :) * q_dot;\n' ...
                                    'end\n']);
        end
    end
    
    add_or_get_from(subsys, 'From_H_torque', 'H_IT', [1100, 1645, 1160, 1665]);
    add_or_get_from(subsys, 'From_Jg_torque1', 'Jg_mat', [1100, 1675, 1160, 1695]);
    add_or_get_from(subsys, 'From_qdot_torque1', 'q_dot', [1100, 1705, 1160, 1725]);
    safe_add_line(subsys, 'From_H_torque/1', [ee_helper_name '/1']);
    safe_add_line(subsys, 'From_Jg_torque1/1', [ee_helper_name '/2']);
    safe_add_line(subsys, 'From_qdot_torque1/1', [ee_helper_name '/3']);
    
    % 4. Wire tau_nom = tau_null (from Manual Switch) directly to S-Function input 9
    add_or_get_from(subsys, 'From_taunull_nom', 'tau_null', [1250, 1815, 1330, 1835]);
    % Remove legacy Sum_tau_nom and From_tautask_nom if present
    if ~isempty(find_system(subsys, 'SearchDepth', 1, 'Name', 'From_tautask_nom'))
        lh_from = get_param([subsys '/From_tautask_nom'], 'LineHandles');
        if lh_from.Outport ~= -1, delete_line(lh_from.Outport); end
        delete_block([subsys '/From_tautask_nom']);
    end
    if ~isempty(find_system(subsys, 'SearchDepth', 1, 'Name', 'Sum_tau_nom'))
        lh_snom = get_param([subsys '/Sum_tau_nom'], 'LineHandles');
        if isfield(lh_snom, 'Inport')
            for p=1:length(lh_snom.Inport)
                if lh_snom.Inport(p) ~= -1, delete_line(lh_snom.Inport(p)); end
            end
        end
        if isfield(lh_snom, 'Outport') && lh_snom.Outport ~= -1
            delete_line(lh_snom.Outport);
        end
        delete_block([subsys '/Sum_tau_nom']);
    end
    
    % 5. S-Function block for Torque CBF (delete and recreate to refresh 13 ports)
    sfun_name = 'S-Function Torque CBF';
    sfun_path = [subsys '/' sfun_name];
    if ~isempty(find_system(subsys, 'SearchDepth', 1, 'Name', sfun_name))
        lh = get_param(sfun_path, 'LineHandles');
        if isfield(lh, 'Inport'), for p = 1:length(lh.Inport), if lh.Inport(p) ~= -1, delete_line(lh.Inport(p)); end; end; end
        if isfield(lh, 'Outport'), for p = 1:length(lh.Outport), if lh.Outport(p) ~= -1, delete_line(lh.Outport(p)); end; end; end
        delete_block(sfun_path);
    end
    add_block('simulink/User-Defined Functions/S-Function', sfun_path, ...
              'FunctionName', 'sfun_torque_cbf', ...
              'Position', [1400, 1580, 1650, 1960]);
    
    % From blocks for S-Function inputs
    add_or_get_from(subsys, 'From_q_sfun', 'q', [1300, 1590, 1360, 1610]);
    add_or_get_from(subsys, 'From_dq_sfun', 'q_dot', [1300, 1620, 1360, 1640]);
    add_or_get_from(subsys, 'From_M_sfun', 'M_mat', [1300, 1710, 1360, 1730]);
    add_or_get_from(subsys, 'From_C_sfun', 'C_mat', [1300, 1740, 1360, 1760]);
    add_or_get_from(subsys, 'From_J_sfun', 'Jg_mat', [1300, 1765, 1360, 1785]);
    add_or_get_from(subsys, 'From_dJ_sfun', 'Jg_dot_mat', [1300, 1790, 1360, 1810]);
    add_or_get_from(subsys, 'From_g_sfun', 'g_vec', [1300, 1835, 1360, 1855]);
    
    % Constant blocks
    const_fext = [subsys '/Constant_Fext'];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', 'Constant_Fext'))
        add_block('simulink/Sources/Constant', const_fext, ...
                  'Value', 'zeros(6,1)', 'Position', [1300, 1860, 1360, 1880]);
    end
    
    const_usefext = [subsys '/Constant_use_fext'];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', 'Constant_use_fext'))
        add_block('simulink/Sources/Constant', const_usefext, ...
                  'Value', '0', 'Position', [1300, 1885, 1360, 1905]);
    end
    
    const_params = [subsys '/Constant_torque_params'];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', 'Constant_torque_params'))
        add_block('simulink/Sources/Constant', const_params, ...
                  'Value', 'cbf_torque_params', 'Position', [1250, 1915, 1360, 1935]);
    end
    
    % Connect 13 Inports to S-Function
    safe_add_line(subsys, 'From_q_sfun/1', [sfun_name '/1']);
    safe_add_line(subsys, 'From_dq_sfun/1', [sfun_name '/2']);
    safe_add_line(subsys, [ee_helper_name '/1'], [sfun_name '/3']);
    safe_add_line(subsys, [ee_helper_name '/2'], [sfun_name '/4']);
    safe_add_line(subsys, 'From_M_sfun/1', [sfun_name '/5']);
    safe_add_line(subsys, 'From_C_sfun/1', [sfun_name '/6']);
    safe_add_line(subsys, 'From_J_sfun/1', [sfun_name '/7']);
    safe_add_line(subsys, 'From_dJ_sfun/1', [sfun_name '/8']);
    safe_add_line(subsys, 'From_taunull_nom/1', [sfun_name '/9']);
    safe_add_line(subsys, 'From_g_sfun/1', [sfun_name '/10']);
    safe_add_line(subsys, 'Constant_Fext/1', [sfun_name '/11']);
    safe_add_line(subsys, 'Constant_use_fext/1', [sfun_name '/12']);
    safe_add_line(subsys, 'Constant_torque_params/1', [sfun_name '/13']);
    
    % Update Sum block to 3 inputs: +++ (1: tau_null via Manual Switch, 2: g_vec, 3: tau_cbf)
    sum_blk = [subsys '/Sum'];
    set_param(sum_blk, 'Inputs', '+++');
    
    % Connect tau_cbf (Outport 1 of S-Function) to Sum input 3
    safe_add_line(subsys, [sfun_name '/1'], 'Sum/3');
    
    % Visualizer for Outport 2 (telemetry)
    vis_name = 'CBF_Telemetry_Visualizer';
    vis_path = [subsys '/' vis_name];
    if isempty(find_system(subsys, 'SearchDepth', 1, 'Name', vis_name))
        add_torque_visualizer();
    else
        safe_add_line(subsys, [sfun_name '/2'], [vis_name '/1']);
    end
    
    save_system(model);
    bdclose(model);
    disp('Successfully configured Controller_torque_cbf.slx!');
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
