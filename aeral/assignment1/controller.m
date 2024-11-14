function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% FILL IN YOUR CODE HERE
e  = s_des(1) - s(1);
e1 = s_des(2) - s(2);
z_des2 = 0;

Kp = 195.24;
Kv = 16;
% u = max( params.u_min ...
%        , min( params.mass * (z_des2 + Kp * e + Kv * e1 + params.gravity) ...
%             , params.u_max));
u = params.mass * (z_des2 + Kp * e + Kv * e1 + params.gravity);

end

