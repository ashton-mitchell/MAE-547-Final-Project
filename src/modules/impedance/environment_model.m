function he = environment_model(x, v, input_sim)
    he = zeros(6, 1);

    if ~input_sim.environment_enabled
        return;
    end

    pos = x(4:6);
    vel = v(4:6);

    % x wall only
    K = input_sim.stiffness(1);
    D = input_sim.damping(1);
    x_wall = input_sim.surface_pose(1);

    e = pos(1) - x_wall;

    if e > 0
        contact_vel = vel(1);
        he(4) = -(K * e+ D * max(contact_vel, 0));
    end
end