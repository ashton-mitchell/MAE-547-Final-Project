function qdd = forward_dynamics(q, qd, tau, robot)
    [M, C, g] = dynamics_model(q, qd, robot);
    qdd = M \ (tau - C - g);
end