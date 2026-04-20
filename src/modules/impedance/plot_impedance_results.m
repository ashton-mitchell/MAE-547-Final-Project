function plot_impedance_results(input, output)
    N = input.robot.n_joints;
    t = output.time;
    ee = output.end_effector.position;
    x_des = input.trajectory.x_des(:, 1:3);
    contact_F = output.he(:,4:6);

    sp = input.sim.surface_pose;

    figure('Name', sprintf('%d-DOF Impedance Control', N), ...
        'Position', [50 80 1400 800], 'Color', 'w');

    ax_lbl = {'X','Y','Z'};

    % Desired pos vs Actual pos
    for k = 1:3
        subplot(2,3,k);
        plot(t, x_des(:,k), 'g--'); 
        hold on;
        plot(t, ee(:,k), 'b-');
        ylabel(sprintf('%s (m)', ax_lbl{k}));
        title(sprintf('EE %s: Desired vs Actual', ax_lbl{k}));
        legend('Desired','Actual','Location','best');
        grid on;
        if k==1 && input.sim.environment_enabled
            yline(sp(1),'r-.','Wall');
            legend('Desired','Actual', 'Wall', 'Location','best');
        else
            legend('Desired','Actual','Location','best');
        end        
    end

    % Contact force
    for k = 1:3
        subplot(2,3,3+k);
        plot(t, contact_F(:,k), 'k-');
        ylabel(sprintf('F_%s (N)', lower(ax_lbl{k})));
        title(sprintf('Contact Force — %s', ax_lbl{k}));
        xlabel('Time (s)'); grid on;
    end
end
