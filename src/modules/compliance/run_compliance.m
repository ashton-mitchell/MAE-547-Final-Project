function output = run_compliance(input)

    N = input.robot.n_joints;
    t = 0:input.sim.dt:input.sim.t_final;

    output.time = t;
    output.q   = zeros(length(t), N);
    output.qd  = zeros(length(t), N);
    output.qdd = zeros(length(t), N);

end