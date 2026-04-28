clear; clc; close all;

thisDir = fileparts(mfilename('fullpath'));
addpath(thisDir);
addpath(fullfile(thisDir, '../../core'));
addpath(fullfile(thisDir, '../impedance'));

robot_dof = 4;

input = get_robot_preset(robot_dof);

input.sim.dt = 0.01;
input.sim.t_final = 1.0;
input.sim.environment_enabled = true;


input.sim.surface_pose(1) = 0.0;

input = trajectory_generation(input, robot_dof);

input.control.Kp_compliance = 30 * ones(robot_dof, 1);
input.control.Kd_compliance = 35 * ones(robot_dof, 1);
input.control.include_gravity = true;

output = run_compliance(input);

plot_compliance_results(input, output);
