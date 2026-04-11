function [M, C, g] = dynamics_model(q, qd, robot)
% input:
%   q   : Nx1
%   qd  : Nx1
%   robot : struct from GUI
%
% output:
%   M : NxN inertia matrix
%   C : Nx1 Coriolis/centrifugal vector
%   g : Nx1 gravity vector

    N = robot.n_joints;

    M = zeros(N,N);
    C = zeros(N,1);
    g = zeros(N,1);

    g_vec = [0; 0; -9.81];

    kin = kinematics(q, robot);

    % build M and g
    for i = 1:N
        % Jacobian for COM of link i
        [Jv, Jw] = jacobian(kin, robot, i);

        % mass and inertia
        m_i = robot.masses(i);

        % inertia
        I_diag = robot.inertia(i,:);
        I_i = diag(I_diag);

        % Rotation of link i
        R_i = kin.T_all(1:3,1:3,i);

        % contribution of inertia matrix
        M = M + m_i * (Jv' * Jv) + Jw' * (R_i * I_i * R_i') * Jw;

        % grav contribution
        g = g + Jv' * (m_i * g_vec);
    end

    % coriolis / centrifugal
    eps = 1e-6;

    for k = 1:N
        dq = zeros(N,1);
        dq(k) = eps;

        M_plus  = mass_matrix_only(q + dq, robot);
        M_minus = mass_matrix_only(q - dq, robot);

        dM_dqk = (M_plus - M_minus) / (2 * eps);

        C = C + dM_dqk * qd * qd(k);
    end
end

function M = mass_matrix_only(q, robot)

    N = robot.n_joints;
    M = zeros(N,N);

    kin = kinematics(q, robot);

    for i = 1:N
        [Jv, Jw] = jacobian(kin, robot, i);

        m_i = robot.masses(i);

        I_diag = robot.inertia(i,:);
        I_i = diag(I_diag);

        R_i = kin.T_all(1:3,1:3,i);

        M = M + m_i * (Jv' * Jv) + Jw' * (R_i * I_i * R_i') * Jw;
    end
end