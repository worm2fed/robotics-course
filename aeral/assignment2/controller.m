function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], 
%   state.vel = [y_dot; z_dot], 
%   state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], 
%   des_state.vel = [y_dot; z_dot], 
%   des_state.acc = [y_ddot; z_ddot]
%
%   params: robot parameters
%   gravity	     9.8100
%   mass	     0.1800
%   Ixx	         2.5000e-04
%   arm_length	 0.0860
%   minF	     0
%   maxF	     3.5316

%   Using these current and desired states, you have to compute the desired
%   controls

% FILL IN YOUR CODE HERE

Kp_z = 35;
Kv_z = 3.5;
u1 = params.mass * (params.gravity + des_state.acc(2) ...
                  + Kv_z * (des_state.vel(2) - state.vel(2)) ...
                  + Kp_z * (des_state.pos(2) - state.pos(2)) );

Kp_y = 20;
Kv_y = 8;
phi_c = (-1 / params.gravity) * (des_state.acc(1) ...
                               + Kv_y * (des_state.vel(1) - state.vel(1)) ...
                               + Kp_y * (des_state.pos(1) - state.pos(1)) );
phi_c_dot = 0;
phi_c_ddot = 0;

Kp_phi = 330;
Kv_phi = 17;
u2 = params.Ixx * (phi_c_ddot ...
                 + Kv_phi * (phi_c_dot - state.omega(1) ...
                 + Kp_phi * (phi_c - state.rot)) );

end

