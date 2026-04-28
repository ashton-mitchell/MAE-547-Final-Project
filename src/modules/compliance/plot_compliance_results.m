function plot_compliance_results(input, output)
    
    N = input.robot.n_joints;
    t = output.time;
    ee = output.end_effector.position;
    x_des = output.end_effector.desired_position;
    contact_F = output.contact_force;

    sp = input.sim.surface_pose;

    figure('Name', sprintf('%d-DOF Compliance Control', N), ...
        'Position', [50 80 1400 800], 'Color', 'w');

    ax_lbl = {'X','Y','Z'};

    for k = 1:3
        subplot(2,3,k);
        plot(t, x_des(:,k), 'g--', 'LineWidth', 1.2);
        hold on;
        plot(t, ee(:,k), 'b-', 'LineWidth', 1.2);
        ylabel(sprintf('%s (m)', ax_lbl{k}));
        title(sprintf('EE %s: Desired vs Actual', ax_lbl{k}));
        grid on;

        if k == 1 && input.sim.environment_enabled
            yline(sp(1), 'r-.', 'Wall');
            legend('Desired','Actual','Wall','Location','best');
        else
            legend('Desired','Actual','Location','best');
        end
    end

    for k = 1:3
        subplot(2,3,3+k);
        plot(t, contact_F(:,k), 'k-', 'LineWidth', 1.2);
        ylabel(sprintf('F_%s (N)', lower(ax_lbl{k})));
        title(sprintf('Contact Force — %s', ax_lbl{k}));
        xlabel('Time (s)');
        grid on;
    end

end
