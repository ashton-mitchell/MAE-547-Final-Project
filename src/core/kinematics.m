function kin = kinematics(q, robot)
% input:
%   q      : Nx1 joint vector
%   robot  : struct from GUI (a, alpha, d, theta, joint_types, COM)
%
% output:
%   kin.T_all     : 4x4xN transforms (base -> link i)
%   kin.T_ee      : 4x4 end-effector transform
%   kin.p_ee      : 3x1 end-effector position
%   kin.p_com     : 3xN COM positions (each column is a link COM)
%   kin.z_axes    : 3xN joint axes (in world)
%   kin.p_origins : 3xN joint origins (in world)

    N = robot.n_joints;

    T_all = zeros(4,4,N);
    z_axes = zeros(3,N);
    p_origins = zeros(3,N);
    p_com = zeros(3,N);

    % Base frame
    T = eye(4);

    for i = 1:N
        z_axes(:,i) = T(1:3,3);
        p_origins(:,i) = T(1:3,4);

        if robot.joint_types(i) == "R" % revolute
            theta = robot.theta(i) + q(i);
            d     = robot.d(i);
        else % prismatic
            theta = robot.theta(i);
            d     = robot.d(i) + q(i);
        end

        % DH transform
        A = dh_transform(robot.a(i), robot.alpha(i), d, theta);

        % Update transform
        T = T * A;

        % Store
        T_all(:,:,i) = T;

        % COM position
        R = T(1:3,1:3);
        p = T(1:3,4);
        p_com(:,i) = R * robot.COM(i,:)' + p;
    end

    % End-effector
    T_ee = T_all(:,:,N);
    p_ee = T_ee(1:3,4);

    kin.T_all = T_all;
    kin.T_ee = T_ee;
    kin.p_ee = p_ee;
    kin.p_com = p_com;
    kin.z_axes = z_axes;
    kin.p_origins = p_origins;

end

function T = dh_transform(a, alpha, d, theta)

    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];

end
