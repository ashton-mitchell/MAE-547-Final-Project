function input = create_sim_input()
    input.robot.n_joints = 0;

    input.robot.joint_types = [];   % Nx1 string

    % DH parameters (Nx1 each)
    input.robot.a = [];
    input.robot.alpha = [];
    input.robot.d = [];
    input.robot.theta = [];

    % dynamics
    input.robot.masses = [];        % Nx1
    input.robot.COM = [];        % Nx3
    input.robot.inertia = [];       % Nx3 (diagonal)

    % limits
    input.robot.joint_min = [];     % Nx1
    input.robot.joint_max = [];     % Nx1

    input.sim.dt = 0;
    input.sim.t_final = 0;

    input.sim.q0 = [];             % Nx1
    input.sim.qd0 = [];             % Nx1

    input.sim.external_tau = [];    % Nx1
    input.sim.external_amplitude = [];
    input.sim.external_frequency = [];

    input.sim.environment_enabled = false;

    input.sim.surface_pose = [];    % Nx1 or scalar
    input.sim.stiffness = [];    % Nx1
    input.sim.damping = [];    % Nx1

    input.control.mode = 'Dynamics';

    % compliance
    input.control.kp = [];          % Nx1
    input.control.kd = [];          % Nx1
    input.control.gravity_comp = false;

    % impedance
    input.control.Md = [];          % Nx1
    input.control.Bd = [];          % Nx1
    input.control.Kd = [];          % Nx1

    input.control.use_inverse_dynamics = false;
    input.control.include_gravity = false;
    input.control.include_coriolis = false;

    input.trajectory.type = 'Constant';

    input.trajectory.x_des = [];  % Mx3
    input.trajectory.xd_des = [];  % Mx3
    input.trajectory.xdd_des = [];  % Mx3

    input.trajectory.amplitude = []; % Mx1
    input.trajectory.frequency = []; % Mx1
    input.trajectory.step_time = []; % Mx1

end