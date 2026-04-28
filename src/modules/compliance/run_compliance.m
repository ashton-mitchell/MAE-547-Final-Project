function output = run_compliance(input)

    dt = input.sim.dt;
    t = 0:dt:input.sim.t_final;
    nSteps = length(t);
    n = input.robot.n_joints;

    if ~isfield(input, 'trajectory') || ~isfield(input.trajectory, 'x_des')
        input = trajectory_generation(input, n);
    end

    q_mat = zeros(nSteps, n);
    qd_mat = zeros(nSteps, n);
    qdd_mat = zeros(nSteps, n);
    tau_mat = zeros(nSteps, n);
    x_mat = zeros(nSteps, 6);
    x_des_mat = zeros(nSteps, 3);
    he_mat = zeros(nSteps, 6);

    q_curr = input.sim.q0(:);
    qd_curr = input.sim.qd0(:);

    q_ref = q_curr;
    qd_ref = zeros(n, 1);

    for i = 1:nSteps

        kin = kinematics(q_curr, input.robot);
        x_pos = kin.p_ee;

        J_pos = ee_jacobian(kin, input.robot);
        J_full = [zeros(3,n); J_pos];

        x_curr = [zeros(3,1); x_pos];
        v_curr = J_full * qd_curr;

        he = environment_model(x_curr, v_curr, input.sim);

        idx = min(i, size(input.trajectory.x_des, 1));
        x_des = input.trajectory.x_des(idx, 1:3)';

        if isfield(input.trajectory, 'xd_des')
            xd_des = input.trajectory.xd_des(idx, 1:3)';
        else
            xd_des = zeros(3,1);
        end

        kin_ref = kinematics(q_ref, input.robot);
        J_ref = ee_jacobian(kin_ref, input.robot);

        K_task = 8.0;
        qd_ref = damped_pinv(J_ref) * (xd_des + K_task * (x_des - kin_ref.p_ee));
        qd_ref = max(min(qd_ref, 4), -4);

        q_ref = q_ref + qd_ref * dt;

        [M, C, g] = dynamics_model(q_curr, qd_curr, input.robot);

        tau = compliance_controller(q_curr, qd_curr, q_ref, qd_ref, g, input);
        tau = max(min(tau, 400), -400);

        qdd = M \ (tau - C - g + J_full' * he);
        qdd = max(min(qdd, 150), -150);

        q_mat(i,:) = q_curr';
        qd_mat(i,:) = qd_curr';
        qdd_mat(i,:) = qdd';
        tau_mat(i,:) = tau';
        x_mat(i,:) = x_curr';
        x_des_mat(i,:) = x_des';
        he_mat(i,:) = he';

        [q_curr, qd_curr] = integrator(q_curr, qd_curr, qdd, dt);

        if mod(i,50) == 0
            drawnow;
        end
    end

    output.time = t;
    output.q = q_mat;
    output.qd = qd_mat;
    output.qdd = qdd_mat;
    output.tau = tau_mat;
    output.he = he_mat;
    output.contact_force = he_mat(:,4:6);
    output.end_effector.position = x_mat(:,4:6);
    output.end_effector.desired_position = x_des_mat;
    output.end_effector.pose = x_mat;

end

function J = ee_jacobian(kin, robot)

    n = robot.n_joints;
    J = zeros(3,n);
    p_ee = kin.p_ee;

    for j = 1:n
        z = kin.z_axes(:,j);
        p_j = kin.p_origins(:,j);

        if robot.joint_types(j) == "R"
            J(:,j) = cross(z, p_ee - p_j);
        else
            J(:,j) = z;
        end
    end

end

function Jpinv = damped_pinv(J)

    lambda = 1e-3;
    Jpinv = J' / (J * J' + lambda * eye(size(J,1)));

end
