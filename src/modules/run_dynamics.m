function output = run_dynamics(input)

    N = input.robot.n_joints;
    t = 0:input.sim.dt:input.sim.t_final;
    dt = input.sim.dt;

    output.time = t;
    output.q   = zeros(length(t), N);
    output.qd  = zeros(length(t), N);
    output.qdd = zeros(length(t), N);
    %int state
    q = input.sim.q0;
    qd = input.sim.qd0;

    %get tau
    if isempty(input.sim.external_tau)
        tau = zeros(N, 1);
    else
        tau = input.sim.external_tau;
    end

    for i = 1:length(t)
        qdd = forward_dynamics(q, qd, tau, input.robot);

        %1xN rows 
        output.q(i, :)   = q';
        output.qd(i, :)  = qd';
        output.qdd(i, :) = qdd';
        if i < length(t)
            [q, qd] = integrator(q, qd, qdd, dt);
        end
    end

end