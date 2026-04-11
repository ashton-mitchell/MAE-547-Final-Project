function [Jv, Jw] = jacobian(kin, robot, link_idx)
% Computes Jacobian for COM of link_idx

    N = robot.n_joints;

    Jv = zeros(3,N);
    Jw = zeros(3,N);

    p_com = kin.p_com(:,link_idx);

    for j = 1:link_idx
        z = kin.z_axes(:,j);
        p_j = kin.p_origins(:,j);

        if robot.joint_types(j) == "R"
            Jv(:,j) = cross(z, (p_com - p_j));
            Jw(:,j) = z;
        else
            Jv(:,j) = z;
            Jw(:,j) = zeros(3,1);
        end
    end
end