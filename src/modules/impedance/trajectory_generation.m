function input = trajectory_generation(input, N)
    robot = build_robot(input.robot);
    tform_init = getTransform(robot, input.sim.q0', sprintf('body%d',N));
    x_init = tform_init(1:3,4);

    t = (0:input.sim.dt:input.sim.t_final)';
    nS = length(t);

    t_hit = 0.5;
    x_wall = input.sim.surface_pose(1);
    x_wall_des = x_wall + 0.05;
    y_slide = 0.5;
    
    input.trajectory.x_des  = zeros(nS, 6);
    input.trajectory.xd_des = zeros(nS, 6);
    
    % trajectory generation
    for k = 1:nS
        ct = t(k);        
        if ct <= t_hit % initial pos -> x wall
            r = ct / t_hit;
            s = 10*r^3-15*r^4+6*r^5;
            ds = (30*r^2-60*r^3+30*r^4)/t_hit;

            input.trajectory.x_des(k,1) = x_init(1) + s*(x_wall - x_init(1));
            input.trajectory.x_des(k,2) = x_init(2);
            input.trajectory.x_des(k,3) = x_init(3);
            
            input.trajectory.xd_des(k,1) = ds*(x_wall - x_init(1));            
        else % interact with x-wall & y slide            
            T_rem = input.sim.t_final - t_hit;
            r = (ct - t_hit)/T_rem;
            s = 10*r^3 - 15*r^4 + 6*r^5;
            ds = (30*r^2 - 60*r^3 + 30*r^4)/T_rem;
            
            input.trajectory.x_des(k,1) = x_wall + s*(x_wall_des - x_wall);
            input.trajectory.x_des(k,2) = x_init(2) + s*(y_slide - x_init(2));
            input.trajectory.x_des(k,3) = x_init(3);
            
            input.trajectory.xd_des(k,1) = ds*(x_wall_des - x_wall);
            input.trajectory.xd_des(k,2) = ds*(y_slide - x_init(2));
        end
    end
end