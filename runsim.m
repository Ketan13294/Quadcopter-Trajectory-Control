clc;
close all;
clear;

addpath('trajectories')
addpath('Planner')
addpath('utils');
% Trajectory generation with waypoints
trajhandle = @traj_generator;

% start and end goal defined.
start = [1,1,2,1,0,0,0];
goal = [5.5,5.5,2,1,0,0,0];

% waypoints calculated from the planner
waypoints = pathPlanner(start,goal,1)';

% t = [0.1:0.1:10]';
% waypoints = [0,0,0;
%     [0.02*t.*t,0.02*t.*t,0.02*t.*t];
%     [2.0+0.1*t,2+0.1*t,2.0+0.0*t]]';

disp('Path Planned!')
%%
trajhandle([],[],waypoints);

%% controller
controlhandle = @controller;

% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
[t, state] = simulation_3d(trajhandle, controlhandle);
