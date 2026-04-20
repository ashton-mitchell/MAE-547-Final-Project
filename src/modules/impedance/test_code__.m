clear; clc; close all;
robot_dof = 4;
input = get_robot_preset(robot_dof);
input = trajectory_generation(input, robot_dof);
output = run_impedance(input);
nSteps = length(output.time);

%%
plot_impedance_results_2(input, output)

%%
animate_impedance_2(input, output)
