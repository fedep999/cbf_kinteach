function add_torque_visualizer()
    model = 'Controller_torque_cbf';
    load_system(model);
    sub = [model '/task space controller'];
    
    term_path = [sub '/Terminator_Torque_CBF_Telem'];
    pos = [1700, 1820, 1720, 1840];
    if ~isempty(find_system(sub, 'SearchDepth', 1, 'Name', 'Terminator_Torque_CBF_Telem'))
        pos = get_param(term_path, 'Position');
        lh = get_param(term_path, 'LineHandles');
        if lh.Inport ~= -1, delete_line(lh.Inport); end
        delete_block(term_path);
    end

    vis_name = 'CBF_Telemetry_Visualizer';
    vis_path = [sub '/' vis_name];
    if ~isempty(find_system(sub, 'SearchDepth', 1, 'Name', vis_name))
        lh = get_param(vis_path, 'LineHandles');
        if isfield(lh, 'Inport') && lh.Inport ~= -1, delete_line(lh.Inport); end
        delete_block(vis_path);
    end

    add_block('simulink/Ports & Subsystems/Subsystem', vis_path, ...
              'Position', [pos(1), pos(2)-15, pos(1)+140, pos(2)+55]);

    % Inside subsystem, delete default In1/Out1 if present
    in1 = [vis_path '/In1'];
    if ~isempty(find_system(vis_path, 'SearchDepth', 1, 'Name', 'In1')), delete_block(in1); end
    out1 = [vis_path '/Out1'];
    if ~isempty(find_system(vis_path, 'SearchDepth', 1, 'Name', 'Out1')), delete_block(out1); end

    % Add Inport
    add_block('simulink/Sources/In1', [vis_path '/telem'], 'Position', [30, 200, 60, 220]);

    % Add Demux (8 channels)
    % 1: is_active
    % 2: zeta
    % 3: dzeta
    % 4: v_min
    % 5: barrier_B
    % 6: f_n_scalar
    % 7: slack_delta
    % 8: qp_cost
    add_block('simulink/Signal Routing/Demux', [vis_path '/Demux'], ...
              'Outputs', '8', 'Position', [120, 50, 125, 370]);
    add_line(vis_path, 'telem/1', 'Demux/1', 'autorouting', 'on');

    % Scope 1: Distance zeta(t)
    add_block('simulink/Sinks/Scope', [vis_path '/Scope_Distance_zeta'], ...
              'Position', [280, 55, 330, 85]);
    add_line(vis_path, 'Demux/2', 'Scope_Distance_zeta/1', 'autorouting', 'on');

    % Scope 2: Velocity & Safe Envelope (dzeta vs v_min)
    add_block('simulink/Signal Routing/Mux', [vis_path '/Mux_PhaseSpace'], ...
              'Inputs', '2', 'Position', [220, 100, 225, 150]);
    add_line(vis_path, 'Demux/3', 'Mux_PhaseSpace/1', 'autorouting', 'on'); % dzeta
    add_line(vis_path, 'Demux/4', 'Mux_PhaseSpace/2', 'autorouting', 'on'); % v_min
    add_block('simulink/Sinks/Scope', [vis_path '/Scope_Velocity_vs_Envelope'], ...
              'Position', [280, 110, 330, 140]);
    add_line(vis_path, 'Mux_PhaseSpace/1', 'Scope_Velocity_vs_Envelope/1', 'autorouting', 'on');

    % Scope 3: Barrier Function B(t)
    add_block('simulink/Sinks/Scope', [vis_path '/Scope_Barrier_B'], ...
              'Position', [280, 165, 330, 195]);
    add_line(vis_path, 'Demux/5', 'Scope_Barrier_B/1', 'autorouting', 'on');

    % Scope 4: Normal Braking Force f_n (N)
    add_block('simulink/Sinks/Scope', [vis_path '/Scope_Braking_Force_fn'], ...
              'Position', [280, 220, 330, 250]);
    add_line(vis_path, 'Demux/6', 'Scope_Braking_Force_fn/1', 'autorouting', 'on');

    % Scope 5: Activity & Slack (is_active, slack_delta)
    add_block('simulink/Signal Routing/Mux', [vis_path '/Mux_ActSlack'], ...
              'Inputs', '2', 'Position', [220, 275, 225, 325]);
    add_line(vis_path, 'Demux/1', 'Mux_ActSlack/1', 'autorouting', 'on'); % is_active
    add_line(vis_path, 'Demux/7', 'Mux_ActSlack/2', 'autorouting', 'on'); % slack_delta
    add_block('simulink/Sinks/Scope', [vis_path '/Scope_Activity_and_Slack'], ...
              'Position', [280, 285, 330, 315]);
    add_line(vis_path, 'Mux_ActSlack/1', 'Scope_Activity_and_Slack/1', 'autorouting', 'on');

    % Scope 6: QP Solver Objective Cost
    add_block('simulink/Sinks/Scope', [vis_path '/Scope_QP_Cost'], ...
              'Position', [280, 340, 330, 370]);
    add_line(vis_path, 'Demux/8', 'Scope_QP_Cost/1', 'autorouting', 'on');

    % Connect S-Function Outport 2 to visualizer Inport 1
    add_line(sub, 'S-Function Torque CBF/2', [vis_name '/1'], 'autorouting', 'on');

    save_system(model);
    bdclose(model);
    disp('Successfully created CBF_Telemetry_Visualizer in Controller_torque_cbf!');
end
