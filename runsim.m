close all;
clear;

addpath('trajectories')
addpath('Planner')
addpath('utils');

%% Trajectory generation with waypoints
trajhandle = @traj_generator;

%%
% start and end goal defined.
start = [0.5,0.5,0];
goal = [6,5,0];

%%
% waypoints calculated from the planner
waypoints = pathPlanner(start,goal,0)

%%

trajhandle([],[],waypoints);

%% controller
controlhandle = @controller;


% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
[t, state] = simulation_3d(trajhandle, controlhandle);
