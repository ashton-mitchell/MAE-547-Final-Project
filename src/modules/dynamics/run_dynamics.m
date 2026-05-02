function output = run_dynamics(input)

    N = input.robot.n_joints;
    t = 0:input.sim.dt:input.sim.t_final;
    dt = input.sim.dt;

    output.time = t;
    output.q   = zeros(length(t), N);
    output.qd  = zeros(length(t), N);
    output.qdd = zeros(length(t), N);
    output.tau = zeros(length(t), N);
    output.end_effector.position = zeros(length(t), 3);
    %int state
    q = input.sim.q0(:);
    qd = input.sim.qd0(:);

    for i = 1:length(t)
        tau = dynamics_tau(input, t(i), N);
        qdd = forward_dynamics(q, qd, tau, input.robot);
        kin = kinematics(q, input.robot);

        %1xN rows 
        output.q(i, :)   = q';
        output.qd(i, :)  = qd';
        output.qdd(i, :) = qdd';
        output.tau(i, :) = tau';
        output.end_effector.position(i, :) = kin.p_ee';
        if i < length(t)
            [q, qd] = integrator(q, qd, qdd, dt);
        end
    end

end

function tau = dynamics_tau(input, time, N)
    if isempty(input.sim.external_tau)
        base_tau = zeros(N, 1);
    else
        base_tau = input.sim.external_tau(:);
    end

    mode = 'Constant Torque';
    if isfield(input.sim, 'external_input') && ~isempty(input.sim.external_input)
        mode = char(input.sim.external_input);
    end

    switch mode
        case 'No External Input'
            tau = zeros(N, 1);
        case 'Sinusoidal Torque'
            amp = expand_vector(input.sim.external_amplitude, N);
            freq = expand_vector(input.sim.external_frequency, N);
            tau = base_tau + amp .* sin(2*pi*freq*time);
        otherwise
            tau = base_tau;
    end
end

function values = expand_vector(values, N)
    if isempty(values)
        values = zeros(N, 1);
    else
        values = values(:);
    end

    if numel(values) < N
        values = [values; zeros(N - numel(values), 1)];
    elseif numel(values) > N
        values = values(1:N);
    end
end
