function tau = compliance_controller(q, qd, q_des, qd_des, g, input)

    n = input.robot.n_joints;

    if isfield(input.control, 'kp') && ~isempty(input.control.kp)
        Kp = diag(input.control.kp(:));
    elseif isfield(input.control, 'Kp_compliance')
        Kp = diag(input.control.Kp_compliance(:));
    else
        Kp = 30 * eye(n);
    end

    if isfield(input.control, 'kd') && ~isempty(input.control.kd)
        Kd = diag(input.control.kd(:));
    elseif isfield(input.control, 'Kd_compliance')
        Kd = diag(input.control.Kd_compliance(:));
    else
        Kd = 35 * eye(n);
    end

    tau = Kp * (q_des - q) + Kd * (qd_des - qd);

    if isfield(input.control, 'gravity_comp')
        include_gravity = input.control.gravity_comp;
    elseif isfield(input.control, 'include_gravity')
        include_gravity = input.control.include_gravity;
    else
        include_gravity = true;
    end

    if include_gravity
        tau = tau + g;
    end

end
