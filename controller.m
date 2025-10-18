function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%m
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

%kp = [120, 110, 75];
%kd = [7, 6.5, 6.5];
%akp = [13.0, 13.6, 10];
%akd = [0.009, 0.009, 0.070];

% kp = [75, 75, 75];
% kd = [7, 6.5, 6.5];
% akp = [13.0, 13.6, 10];
% akd = [0.009, 0.009, 0.070];

% kp = [75, 75, 75];
% kd = [6.5, 6.5, 6.5];
% akp = [14, 14, 14];
% akd = [0.009, 0.009, 0.009];

% kp = [80, 80, 80];
% kd = [8, 8, 8];
% akp = [100, 100, 100];
% akd = [4, 4, 4];

% kp = [180, 180, 180];
% kd = [100, 100, 100];
% akp = [250, 250, 100];
% akd = [100, 100, 10];

% kp = [10, 10, 1];
% kd = [0.01, 0.01, 0.01];
% akp = [1, 1, 10];
% akd = [0.01, 0.01, 0.01];

kp = [24, 24, 750];
kd = [9.2, 9.2, 450];
akp = [80, 80, 80];
akd = [8.6, 8.6, 4.8];

% Unit tangent to trajectory 
tcap = des_state.vel/norm(des_state.vel);
% Unit normal to trajectory
ncap = des_state.acc/norm(des_state.acc);
% Unit normal to the tangent and normal to trajectory
bcap = cross(tcap, ncap);

% Position error in world frame
delta_pos = (des_state.pos-state.pos);

% Position Error
% If normal to trajectory is NaN,
% then position error is along world frame,
% if normal to trajectory is not NaN,
% position is along the trajectories normals and tangents.
%
if(any(isnan(bcap)))
    err_p = delta_pos;
else
    err_p = (delta_pos'*ncap)*ncap + (delta_pos'*bcap)*bcap + (delta_pos'*tcap)*tcap;
end

% Velocity error
err_v = des_state.vel-state.vel;

% I/O linearized acceleration
r1ddot = des_state.acc(1) + kd(1)*err_v(1) + kp(1)*err_p(1);
r2ddot = des_state.acc(2) + kd(2)*err_v(2) + kp(2)*err_p(2);
r3ddot = des_state.acc(3) + kd(3)*err_v(3) + kp(3)*err_p(3);

phi_dest = (1/params.gravity)*(r1ddot * sin(des_state.yaw) - r2ddot * cos(des_state.yaw));
theta_dest = (1/params.gravity)*(r1ddot * cos(des_state.yaw) + r2ddot * sin(des_state.yaw));
psi_dest = des_state.yaw;

p_dest = 0;
q_dest = 0;
r_dest = des_state.yawdot;

% Thrust
F = params.mass*(params.gravity +r3ddot);
if F < params.minF
    F = params.minF;
end
if F > params.maxF
    F = params.maxF;
end

% Moment
M = zeros(3,1);
M(1) = akp(1)*(phi_dest - state.rot(1))   + akd(1)*(p_dest - state.omega(1));
M(2) = akp(2)*(theta_dest - state.rot(2)) + akd(2)*(q_dest - state.omega(2));
M(3) = akp(3)*(psi_dest - state.rot(3))   + akd(3)*(r_dest - state.omega(3));

% =================== Your code ends here ===================

end