function tau = impedance_controller(qd, x_curr, J, dJ, B, C_dq, G, he, input, x_des, xd_des)
    n = input.robot.n_joints;
    
    J_pos = J(4:6,:);
    dJ_pos = dJ(4:6, :);
    e = x_des(4:6) - x_curr(4:6);
    ed = xd_des(4:6) - J_pos*qd;
    
    Md = diag(cartesian_gain(input.control.Md, true));
    Bd = diag(cartesian_gain(input.control.Bd, false));
    Kd = diag(cartesian_gain(input.control.Kd, false));

    he_pos = he(4:6);

    xdd_ref = Md \ (Kd*e + Bd*ed + he_pos);
    qdd_ref = pinv(J_pos)*(xdd_ref - dJ_pos*qd);

    he_joint = J'*he;

    % Null space damping
    null_dim = n - 3; % total - 3 pos
    if null_dim > 0
        null_damping = 100.0;
        qdd_ref = qdd_ref-null_damping*((eye(n)-pinv(J_pos)*J_pos)* qd); % (I-J^#J))qd
    end

    % Joint velocity damping
    joint_damping = 10.0;
    qdd_ref = qdd_ref - joint_damping*qd;

    % Torque
    if input.control.use_inverse_dynamics
        tau = B * qdd_ref;
    else
        F_task = Kd*e + Bd*ed + he(4:6);
        tau = J_pos'*F_task;
    end

    if input.control.include_gravity
        tau = tau + G;
    end

    if input.control.include_coriolis
        tau = tau + C_dq;
    end
    
    tau = tau - he_joint;    
end

function gain = cartesian_gain(values, replace_zero)
    values = values(:);

    if isempty(values)
        gain = ones(3, 1);
    elseif numel(values) >= 6
        gain = values(4:6);
    elseif numel(values) >= 3
        gain = values(1:3);
    elseif numel(values) == 1
        gain = repmat(values, 3, 1);
    else
        gain = [values; repmat(values(end), 3 - numel(values), 1)];
    end

    if replace_zero
        gain(gain == 0) = 1;
    end
end
