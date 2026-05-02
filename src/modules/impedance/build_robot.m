function robot = build_robot(robot_params)
    n = robot_params.n_joints;
    links(1,n) = Link();
    
    for i = 1:n
        switch robot_params.joint_types(i)
            case "R"
                links(i) = Link([0, robot_params.d(i), robot_params.a(i), robot_params.alpha(i), 0, robot_params.theta(i)], 'standard');
            otherwise
                links(i) = Link([robot_params.theta(i), 0, robot_params.a(i), robot_params.alpha(i), 1, robot_params.d(i)], 'standard');
        end

        links(i).m = robot_params.masses(i);
        links(i).r = robot_params.COM(i,:);
        links(i).I = diag(robot_params.inertia(i,:));
        links(i).Jm = 0;
        links(i).B = 0;
        links(i).Tc = [0 0];

        if isfield(robot_params, 'joint_min') && ...
            isfield(robot_params, 'joint_max') && ...
            ~isempty(robot_params.joint_min) && ...
            ~isempty(robot_params.joint_max) && ...
            numel(robot_params.joint_min) >= i && ...
            numel(robot_params.joint_max) >= i
        
            links(i).qlim = [robot_params.joint_min(i), robot_params.joint_max(i)];
         end
    end

    robot = SerialLink(links, 'name', 'gui_robot');
    robot.gravity = [0 0 -9.81];
end
