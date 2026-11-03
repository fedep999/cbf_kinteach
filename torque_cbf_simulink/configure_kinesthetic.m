function configure_kinesthetic()
    model = 'Controller_torque_cbf';
    load_system(model);
    sub = [model '/task space controller'];
    
    % 1. Disconnect MatrixMultiply1 (tau_task) from Sum
    sum_blk = [sub '/Sum'];
    lh_sum = get_param(sum_blk, 'LineHandles');
    
    % Delete line 4 (tau_cbf) and line 3 (tau_task)
    if length(lh_sum.Inport) >= 4 && lh_sum.Inport(4) ~= -1
        delete_line(lh_sum.Inport(4));
    end
    if length(lh_sum.Inport) >= 3 && lh_sum.Inport(3) ~= -1
        delete_line(lh_sum.Inport(3));
    end
    
    % Set Sum inputs to 3: +++ (1: tau_null, 2: g_vec, 3: tau_cbf)
    set_param(sum_blk, 'Inputs', '+++');
    
    % Reconnect tau_cbf to Sum Inport 3
    add_line(sub, 'S-Function Torque CBF/1', 'Sum/3', 'autorouting', 'on');
    
    % 2. Terminate MatrixMultiply1 (tau_task) so it doesn't leave an open port
    term_tautask = [sub '/Terminator_tau_task'];
    if isempty(find_system(sub, 'SearchDepth', 1, 'Name', 'Terminator_tau_task'))
        add_block('simulink/Sinks/Terminator', term_tautask, ...
                  'Position', [2025, 285, 2045, 305]);
    end
    lh_mm1 = get_param([sub '/MatrixMultiply1'], 'LineHandles');
    % If Goto_tau_task exists, delete it
    if ~isempty(find_system(sub, 'SearchDepth', 1, 'Name', 'Goto_tau_task'))
        lh_gt = get_param([sub '/Goto_tau_task'], 'LineHandles');
        if lh_gt.Inport ~= -1, delete_line(lh_gt.Inport); end
        delete_block([sub '/Goto_tau_task']);
    end
    % Connect MatrixMultiply1 to Terminator_tau_task
    safe_add_line(sub, 'MatrixMultiply1/1', 'Terminator_tau_task/1');
    
    % 3. Remove Sum_tau_nom and From_tautask_nom if present
    if ~isempty(find_system(sub, 'SearchDepth', 1, 'Name', 'From_tautask_nom'))
        lh_from = get_param([sub '/From_tautask_nom'], 'LineHandles');
        if lh_from.Outport ~= -1, delete_line(lh_from.Outport); end
        delete_block([sub '/From_tautask_nom']);
    end
    
    if ~isempty(find_system(sub, 'SearchDepth', 1, 'Name', 'Sum_tau_nom'))
        lh_snom = get_param([sub '/Sum_tau_nom'], 'LineHandles');
        if isfield(lh_snom, 'Inport')
            for p=1:length(lh_snom.Inport)
                if lh_snom.Inport(p) ~= -1, delete_line(lh_snom.Inport(p)); end
            end
        end
        if isfield(lh_snom, 'Outport') && lh_snom.Outport ~= -1
            delete_line(lh_snom.Outport);
        end
        delete_block([sub '/Sum_tau_nom']);
    end
    
    % 4. Connect From_taunull_nom (tag: tau_null) directly to S-Function Torque CBF/9
    % (tau_nom = tau_null, switchable to 0 via Manual Switch)
    add_or_get_from(sub, 'From_taunull_nom', 'tau_null', [1250, 1815, 1330, 1835]);
    safe_add_line(sub, 'From_taunull_nom/1', 'S-Function Torque CBF/9');
    
    save_system(model);
    bdclose(model);
    disp('Successfully configured pure kinesthetic control in Controller_torque_cbf.slx!');
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
