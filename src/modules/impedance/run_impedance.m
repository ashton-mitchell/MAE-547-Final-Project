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
    x_des_mat = zeros(nSteps, 3);
    he_mat = zeros(nSteps, 6);

    q_curr = input.sim.q0(:);
    qd_curr = input.sim.qd0(:);

    robot = build_robot(input.robot);

    for i = 1:nSteps
        q_mat(i,:)  = q_curr';
        qd_mat(i,:) = qd_curr';

        q_row = q_curr';
        qd_row = qd_curr';

        B = robot.inertia(q_row);
        C_dq = robot.coriolis(q_row, qd_row) * qd_curr;
        G = robot.gravload(q_row)';

        tform = robot.fkine(q_row);
        pos_curr = tform(1:3, 4);
        eul_curr = tr2eul(tform)';
        x_curr   = [eul_curr; pos_curr];
        J0 = robot.jacob0(q_row);
        J = [J0(4:6,:); J0(1:3,:)];
        v_curr = J * qd_curr;

        dt_eps = 1e-5;
        J0_next = robot.jacob0((q_curr + qd_curr*dt_eps)');
        J_next = [J0_next(4:6,:); J0_next(1:3,:)];
        dJ = (J_next - J) / dt_eps;

        he = environment_model(x_curr, v_curr, input.sim);

        [x_des_i, xd_des_i] = trajectory_sample(input.trajectory, i);

        x_mat(i,:)  = x_curr';
        x_des_mat(i,:) = x_des_i(4:6)';
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
    output.end_effector.desired_position = x_des_mat;
    output.end_effector.pose = x_mat;
end

function [x_des, xd_des] = trajectory_sample(traj, i)
    x_des = zeros(6, 1);
    xd_des = zeros(6, 1);

    if isfield(traj, 'x_des') && ~isempty(traj.x_des)
        idx = min(i, size(traj.x_des, 1));
        cols = min(3, size(traj.x_des, 2));
        x_des(4:(3 + cols)) = traj.x_des(idx, 1:cols)';
    end

    if isfield(traj, 'xd_des') && ~isempty(traj.xd_des)
        idx = min(i, size(traj.xd_des, 1));
        cols = min(3, size(traj.xd_des, 2));
        xd_des(4:(3 + cols)) = traj.xd_des(idx, 1:cols)';
    end
end
