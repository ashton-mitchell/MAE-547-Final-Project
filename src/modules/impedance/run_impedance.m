function output = run_impedance(input)
    dt = input.sim.dt;
    t = 0:dt:input.sim.t_final;
    nSteps = length(t);
    n = input.robot.n_joints;

    q_mat = zeros(nSteps, n);
    qd_mat = zeros(nSteps, n);
    qdd_mat = zeros(nSteps, n);
    tau_mat = zeros(nSteps, n);
    x_mat = zeros(nSteps, 6);
    he_mat = zeros(nSteps, 6);

    q_curr = input.sim.q0(:);
    qd_curr = input.sim.qd0(:);

    robot = build_robot(input.robot);
    EEbody = sprintf('body%d', n);

    traj_x_des = [input.trajectory.x_des(:,4:6),  input.trajectory.x_des(:,1:3)];
    traj_xd_des = [input.trajectory.xd_des(:,4:6), input.trajectory.xd_des(:,1:3)];

    for i = 1:nSteps
        q_mat(i,:)  = q_curr';
        qd_mat(i,:) = qd_curr';

        B    = massMatrix(robot, q_curr');
        C_dq = velocityProduct(robot, q_curr', qd_curr')';
        G    = gravityTorque(robot, q_curr')';

        J = geometricJacobian(robot, q_curr', EEbody);
        tform  = getTransform(robot, q_curr', EEbody);

        pos_curr = tform(1:3, 4);
        eul_curr = flip(tform2eul(tform))';
        x_curr   = [eul_curr; pos_curr];
        v_curr = J * qd_curr;

        % Differential Jacobian
        dt_eps = 1e-5;
        J_next = geometricJacobian(robot, (q_curr + qd_curr*dt_eps)', EEbody);
        dJ = (J_next - J) / dt_eps;

        he = environment_model(x_curr, v_curr, input.sim);

        idx = min(i, size(traj_x_des, 1)); 
        x_des_i = traj_x_des(idx, :)';
        xd_des_i = traj_xd_des(idx, :)';

        x_mat(i,:)  = x_curr';
        he_mat(i,:) = he';

        tau = impedance_controller(qd_curr, x_curr, J, dJ, B, C_dq, G, he, input, x_des_i, xd_des_i);
        tau = max(min(tau, 500), -500);
        tau_mat(i,:) = tau';

        qdd = B \ (tau - C_dq - G + J' * he);
        qdd = max(min(qdd, 200), -200);
        qdd_mat(i,:) = qdd';

        qd_curr = qd_curr + qdd * dt;
        q_curr = q_curr + qd_curr * dt;
    end

    output.time = t;
    output.q = q_mat;
    output.qd = qd_mat;
    output.qdd = qdd_mat;
    output.tau = tau_mat;
    output.he = he_mat;
    output.contact_force = he_mat(:, 4:6);
    output.end_effector.position = x_mat(:, 4:6);
    output.end_effector.pose = x_mat;
end