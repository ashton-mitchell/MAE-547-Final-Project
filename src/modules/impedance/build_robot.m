function robot = build_robot(robot_params)
    n = robot_params.n_joints;
    robot = rigidBodyTree;
    robot.DataFormat = 'row';
    
    for i = 1:n
        body  = rigidBody(sprintf('body%d', i));        
        switch robot_params.joint_types(i)
            case "R"
                joint = rigidBodyJoint(sprintf("joint%d", i), 'revolute');
            case "P"
                joint = rigidBodyJoint(sprintf("joint%d", i), 'prismatic');
        end        
        setFixedTransform(joint, [robot_params.a(i), robot_params.alpha(i), robot_params.d(i), robot_params.theta(i)], 'dh');

        body.Joint = joint;
        body.Mass  = robot_params.masses(i);
        body.CenterOfMass = robot_params.COM(i,:);
        body.Inertia = [robot_params.inertia(i,:), 0, 0, 0];
        if i == 1
            addBody(robot, body, 'base');
        else
            addBody(robot, body, sprintf('body%d', i-1));
        end
    end
end
