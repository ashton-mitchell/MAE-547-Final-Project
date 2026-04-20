function animate_impedance(input, output)

    N = input.robot.n_joints;
    t = output.time;
    ee = output.end_effector.position;
    robot = build_robot(input.robot);

    fps = 30;
    skip = max(1, round(1/(input.sim.dt * fps)));
    ai = 1:skip:length(t);
    if ai(end) ~= length(t)
        ai(end+1) = length(t);
    end

    figure('Name','Simulation');
    axis_lim = 3;
    ax = axes;
    hold(ax, 'on');
    axis(ax, 'equal');
    grid(ax, 'on');
    view(ax, [135 25]);
    xlim(ax, [-axis_lim axis_lim]);
    ylim(ax, [-axis_lim axis_lim]);
    zlim(ax, [-0.5 axis_lim]);
    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');

    draw_env(ax, input, axis_lim);

    h_links = gobjects(N,1);
    for i = 1:N
        h_links(i) = plot3(ax, [0 0], [0 0], [0 0], 'k-', 'LineWidth', 4);
    end

    h_joints = scatter3(ax, 0, 0, 0, 80, 'k', 'filled');
    h_ee = scatter3(ax, 0, 0, 0, 120, 'r', 'filled');
    h_trail = plot3(ax, 0, 0, 0, 'b-');

    for k = 1:length(ai)
        idx = ai(k);
        q_row = output.q(idx,:);

        joint_pos = zeros(3, N+1);
        for i = 1:N
            T = getTransform(robot, q_row, sprintf('body%d', i));
            joint_pos(:,i+1) = T(1:3,4);
        end

        for i = 1:N
            set(h_links(i), 'XData', joint_pos(1,i:i+1), 'YData', joint_pos(2,i:i+1), 'ZData', joint_pos(3,i:i+1));
        end

        set(h_joints, 'XData', joint_pos(1,:), 'YData', joint_pos(2,:), 'ZData', joint_pos(3,:));
        set(h_ee, 'XData', joint_pos(1,end), 'YData', joint_pos(2,end), 'ZData', joint_pos(3,end));
        set(h_trail, 'XData', ee(1:idx,1), 'YData', ee(1:idx,2), 'ZData', ee(1:idx,3));
        drawnow;
    end
end

function draw_env(ax, input, axis_lim)
    sp = input.sim.surface_pose;
    xlim_m = axis_lim;
    ylim_m = axis_lim;
    
    % Draw x wall
    [WY, WZ] = meshgrid(linspace(-xlim_m, xlim_m, 2), linspace(-ylim_m, ylim_m, 2));
    surf(ax, sp(1)*ones(size(WY)), WY, WZ, 'FaceColor', 'r', 'FaceAlpha', 0.15, 'EdgeColor', [0.7 0 0], 'EdgeAlpha', 0.4);

    % Draw floor
    [FX, FY] = meshgrid(linspace(-xlim_m, xlim_m, 2), linspace(-ylim_m, ylim_m, 2));
    FZ = sp(3)*ones(size(FX));
    surf(ax, FX, FY, FZ, 'FaceColor', 'k', 'FaceAlpha', 0.2, 'EdgeColor', [0.5 0.35 0.2], 'EdgeAlpha', 0.3);
end