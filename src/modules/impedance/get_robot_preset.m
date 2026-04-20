function input = get_robot_preset(dof)
    switch dof
        case 3, input = preset_3dof();
        case 4, input = preset_4dof();
        case 5, input = preset_5dof();
        case 6, input = preset_6dof();
        case 7, input = preset_7dof();
        case 8, input = preset_8dof();
        case 9, input = preset_9dof();
        otherwise, error('Supported: 3-9 DOF');
    end
end

function input = fill_common(input, N)
    input.robot.n_joints = N;
    input.sim.dt = 0.001;
    input.sim.t_final = 3.0;
    input.sim.qd0 = zeros(N,1);
    input.sim.environment_enabled = true;
    input.control.use_inverse_dynamics = true;
    input.control.include_gravity = true;
    input.control.include_coriolis = true;
end

%% 3-DOF
function input = preset_3dof()
    N=3; input=struct(); input=fill_common(input,N);
    input.robot.joint_types = ["R","R","R"];
    input.robot.a = [0.10; 0.80; 0.60];
    input.robot.alpha = [pi/2; 0; 0];
    input.robot.d = [0.25; 0; 0]; input.robot.theta = [0;0;0];
    input.robot.masses = [2; 1.5; 1];
    input.robot.COM = [0.05 0 0.12; 0.40 0 0; 0.30 0 0];
    input.robot.inertia = [0.15 0.15 0.05; 0.10 0.10 0.03; 0.05 0.05 0.02];
    input.sim.q0 = [-3/4*pi; pi/4; -pi/6];
    input.sim.surface_pose = [1.0;0;0; 0;0;0];
    input.sim.stiffness = [5000;0;0; 0;0;0]; input.sim.damping = [100;0;0; 0;0;0];
    input.control.Md = [2;2;2; 2;2;2];
    input.control.Bd = [15;15;15; 42;42;42];
    input.control.Kd = [80;80;80; 200;200;200];
end

%% 4-DOF
function input = preset_4dof()
    N=4; input=struct(); input=fill_common(input,N);
    input.robot.joint_types = ["R","R","R","R"];
    input.robot.a = [0.10; 0.40; 0.35; 0.20];
    input.robot.alpha = [pi/2; 0; pi/2; 0];
    input.robot.d = [0.30; 0; 0; 0]; input.robot.theta = zeros(4,1);
    input.robot.masses = [2.5; 2; 1.5; 0.8];
    input.robot.COM = [0.05 0 0.15; 0.20 0 0; 0.17 0 0; 0.10 0 0];
    input.robot.inertia = [0.06 0.06 0.03; 0.04 0.04 0.01; 0.03 0.03 0.008; 0.01 0.01 0.003];
    input.sim.q0 = [-3/4*pi; pi/4; -pi/6; 0];
    input.sim.surface_pose = [0.60;0;0; 0;0;0];
    input.sim.stiffness = [4000;0;0; 0;0;0]; input.sim.damping = [100;0;0; 0;0;0];
    input.control.Md = [2;2;2; 3;3;3];
    input.control.Bd = [15;15;15; 63;63;63];
    input.control.Kd = [80;80;80; 300;300;300];
end

%% 5-DOF
function input = preset_5dof()
    N=5; input=struct(); input=fill_common(input,N);
    input.robot.joint_types = ["R","R","R","R","R"];
    input.robot.a = [0.10; 0.35; 0.30; 0.25; 0.15];
    input.robot.alpha = [pi/2; 0; pi/2; 0; pi/2];
    input.robot.d = [0.30; 0; 0; 0; 0]; input.robot.theta = zeros(5,1);
    input.robot.masses = [3; 2; 1.5; 1; 0.5];
    input.robot.COM = [0.05 0 0.15; 0.17 0 0; 0.15 0 0; 0.12 0 0; 0.07 0 0];
    input.robot.inertia = [0.08 0.08 0.03; 0.04 0.04 0.01; 0.03 0.03 0.008; 0.01 0.01 0.004; 0.005 0.005 0.002];
    input.sim.q0 = [-3/4*pi; pi/6; -pi/6; pi/8; 0];
    input.sim.surface_pose = [0.55;0;0; 0;0;0];
    input.sim.stiffness = [3500;0;0; 0;0;0]; input.sim.damping = [100;0;0; 0;0;0];
    input.control.Md = [2;2;2; 3;3;3];
    input.control.Bd = [15;15;15; 81;81;81];
    input.control.Kd = [80;80;80; 500;500;500];
end

%% 6-DOF
function input = preset_6dof()
    N=6; input=struct(); input=fill_common(input,N);
    input.robot.joint_types = ["R","R","R","R","R","R"];
    input.robot.a = [0.10; 0.35; 0.30; 0.25; 0.20; 0.12];
    input.robot.alpha = [pi/2; 0; pi/2; 0; -pi/2; 0];
    input.robot.d = [0.35; 0; 0; 0; 0; 0]; input.robot.theta = zeros(6,1);
    input.robot.masses = [3.5; 2.5; 2; 1.5; 1; 0.5];
    input.robot.COM = [0.05 0 0.17; 0.17 0 0; 0.15 0 0; 0.12 0 0; 0.10 0 0; 0.06 0 0];
    input.robot.inertia = [0.10 0.10 0.04; 0.06 0.06 0.02; 0.04 0.04 0.01; 0.02 0.02 0.008; 0.01 0.01 0.005; 0.005 0.005 0.002];
    input.sim.q0 = [-3/4*pi; pi/4; -pi/6; pi/8; -pi/8; 0];
    input.sim.surface_pose = [0.55;0;0; 0;0;0];
    input.sim.stiffness = [3000;0;0; 0;0;0]; input.sim.damping = [120;0;0; 0;0;0];
    input.control.Md = [3;3;3; 4;4;4];
    input.control.Bd = [20;20;20; 94;94;94];
    input.control.Kd = [80;80;80; 500;500;500];
end

%% 7-DOF  Md=5, Kd=500, Bd=105, ζ=1.05
%  Palm flat + roll during slide
function input = preset_7dof()
    N=7; input=struct(); input=fill_common(input,N);
    input.sim.t_final = 5.0;
    input.robot.joint_types = repmat("R",1,7);
    input.robot.a = [0.10; 0.30; 0.25; 0.25; 0.20; 0.15; 0.10];
    input.robot.alpha = [pi/2; 0; pi/2; 0; -pi/2; 0; pi/2];
    input.robot.d = [0.35; 0; 0; 0; 0; 0; 0]; input.robot.theta = zeros(7,1);
    input.robot.masses = [4; 3; 2.5; 2; 1.5; 1; 0.5];
    input.robot.COM = [0.05 0 0.17; 0.15 0 0; 0.12 0 0; 0.12 0 0; 0.10 0 0; 0.07 0 0; 0.05 0 0];
    input.robot.inertia = [0.12 0.12 0.04; 0.06 0.06 0.02; 0.05 0.05 0.015; 0.04 0.04 0.01; 0.02 0.02 0.008; 0.01 0.01 0.005; 0.005 0.005 0.002];
    input.sim.q0 = [-3/4*pi; pi/4; 0; -pi/3; pi/6; -pi/8; 0];
    input.sim.surface_pose = [0.65;0;0; 0;0;0];
    input.sim.stiffness = [2500;0;0; 0;0;0]; input.sim.damping = [150;0;0; 0;0;0];
    input.control.Md = [3;3;3; 5;5;5];
    input.control.Bd = [25;25;25; 105;105;105];
    input.control.Kd = [80;80;80; 500;500;500];
end

%% 8-DOF  Md=6, Kd=1000, Bd=163, ζ=1.05
function input = preset_8dof()
    N=8; input=struct(); input=fill_common(input,N);
    input.sim.t_final = 5.0;
    input.robot.joint_types = ["P","R","R","R","R","R","R","R"];
    input.robot.a = [0; 0.10; 0.30; 0.25; 0.25; 0.20; 0.15; 0.10];
    input.robot.alpha = [0; pi/2; 0; pi/2; 0; -pi/2; 0; pi/2];
    input.robot.d = [0; 0.25; 0; 0; 0; 0; 0; 0]; input.robot.theta = zeros(8,1);
    input.robot.masses = [5; 3.5; 2.5; 2; 1.5; 1; 0.8; 0.4];
    input.robot.COM = [0 0 0; 0.05 0 0.12; 0.15 0 0; 0.12 0 0; 0.12 0 0; 0.10 0 0; 0.07 0 0; 0.05 0 0];
    input.robot.inertia = [0.05 0.05 0.05; 0.08 0.08 0.03; 0.05 0.05 0.015; 0.04 0.04 0.01; 0.02 0.02 0.008; 0.01 0.01 0.005; 0.008 0.008 0.003; 0.003 0.003 0.001];
    input.sim.q0 = [0.2; -3/4*pi; pi/4; -pi/6; pi/8; -pi/8; 0; 0];
    input.sim.surface_pose = [0.85;0;0; 0;0;0];
    input.sim.stiffness = [2000;0;0; 0;0;0]; input.sim.damping = [150;0;0; 0;0;0];
    input.control.Md = [4;4;4; 6;6;6];
    input.control.Bd = [25;25;25; 163;163;163];
    input.control.Kd = [80;80;80; 1000;1000;1000];
end

%% 9-DOF  Md=5, Kd=1000, Bd=148, ζ=1.05
function input = preset_9dof()
    N=9; input=struct(); input=fill_common(input,N);
    input.sim.t_final = 5.0;
    input.robot.joint_types = repmat("R",1,9);
    input.robot.a = [0.08; 0.22; 0.20; 0.18; 0.18; 0.15; 0.15; 0.12; 0.08];
    input.robot.alpha = [pi/2; 0; pi/2; 0; -pi/2; 0; pi/2; 0; -pi/2];
    input.robot.d = [0.30; 0; 0; 0; 0; 0; 0; 0; 0]; input.robot.theta = zeros(9,1);
    input.robot.masses = [4; 3; 2.5; 2; 1.5; 1.2; 0.8; 0.5; 0.3];
    input.robot.COM = [0.04 0 0.15; 0.11 0 0; 0.10 0 0; 0.09 0 0; 0.09 0 0; 0.07 0 0; 0.07 0 0; 0.06 0 0; 0.04 0 0];
    input.robot.inertia = [0.10 0.10 0.04; 0.06 0.06 0.02; 0.05 0.05 0.015; 0.04 0.04 0.01; 0.02 0.02 0.008; 0.015 0.015 0.006; 0.008 0.008 0.003; 0.005 0.005 0.002; 0.002 0.002 0.001];
    input.sim.q0 = [-3/4*pi; pi/6; -pi/8; pi/6; -pi/6; pi/8; -pi/8; pi/10; 0];
    input.sim.surface_pose = [0.85;0;0; 0;0;0];
    input.sim.stiffness = [2000;0;0; 0;0;0]; input.sim.damping = [150;0;0; 0;0;0];
    input.control.Md = [3;3;3; 5;5;5];
    input.control.Bd = [25;25;25; 148;148;148];
    input.control.Kd = [80;80;80; 1000;1000;1000];
end