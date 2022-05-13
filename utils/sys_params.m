function params = sys_params()
% SYS_PARAMS basic parameters for the quadrotor

m = 1.2; % kg
g = 9.81; % m/s/s
I = [0.005,   0,          0;
     0,         0.017,   0;
     0,   0,          0.02];

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.gravity = g;

params.arm_length = 0.086;
params.minF = 0.0;
params.maxF = 2.0*m*g;

end
