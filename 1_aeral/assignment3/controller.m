function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
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
%   mass	   0.1800
%   I	       [2.5000e-04,0,2.5500e-06;0,2.3200e-04,0;2.5500e-06,0,3.7380e-04]
%   invI	   [4.0003e+03,0,-27.2892;0,4.3103e+03,0;-27.2892,0,2.6754e+03]
%   gravity	   9.8100
%   arm_length 0.0860
%   minF	   0
%   maxF	   3.5316
%
%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Gains
Kp_pos = [10; 10; 80];
Kd_pos = [10; 10; 35];
Kp_rot = [200; 200; 200]; 
Kd_rot = [.1; .1; .1];

% Translational control
a_des = des_state.acc + Kd_pos .* (des_state.vel - state.vel) ...
                      + Kp_pos .* (des_state.pos - state.pos);

% Thrust
F = params.mass * (params.gravity + a_des(3));

% Rotational control
phi_des   = (1 / params.gravity) ...
          * (a_des(1) * sin(des_state.yaw) - a_des(2) * cos(des_state.yaw));
theta_des = (1 / params.gravity) ...
          * (a_des(1) * cos(des_state.yaw) + a_des(2) * sin(des_state.yaw));
rot_des = [phi_des; theta_des; des_state.yaw];
omega_des = [0; 0; des_state.yawdot];

% Moment
M = Kp_rot .* (rot_des - state.rot) + ...
    Kd_rot .* (omega_des - state.omega);

% =================== Your code ends here ===================

end
