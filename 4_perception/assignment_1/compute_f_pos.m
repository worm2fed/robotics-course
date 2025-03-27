function [ f, pos ] = compute_f_pos( d1_ref, d2_ref, H1, H2, ratio, f_ref )
%% Compute camera focal length and camera position to achieve centain ratio between objects
%
% In this function, we focus on two objects: object A with height H1 and
% d1_ref as distance to camera in 3D world, object B with height H2 and
% d2_ref as distance to camera in 3D world.
% We will keep the size of object A in image the same as before while
% adjusting the size of object B in image.
%
% Input:
% - d1_ref: distance of the first object
% - d2_ref: distance of the second object
% - H1: height of the first object in physical world
% - H2: height of the second object in physical world
% - ratio: ratio between two objects in image coordinate (h1/h2)
% - f_ref: 1 by 1 double, previous camera focal length
% Output:
% - f: 1 by 1, camera focal length
% - pos: 1 by 1, camera position on z axis

% YOUR CODE HERE

% Using the projection equation: u = f * X/Z
% For object A: h1 = f * H1/(d1_ref - pos)
% For object B: h2 = f * H2/(d2_ref - pos)
% We want: ratio = h1/h2 = (f * H1/(d1_ref - pos))/(f * H2/(d2_ref - pos))
% Therefore: ratio = (H1 * (d2_ref - pos))/(H2 * (d1_ref - pos))

% Solve for pos:
% ratio * H2 * (d1_ref - pos) = H1 * (d2_ref - pos)
% ratio * H2 * d1_ref - ratio * H2 * pos = H1 * d2_ref - H1 * pos
% H1 * pos - ratio * H2 * pos = H1 * d2_ref - ratio * H2 * d1_ref
% pos * (H1 - ratio * H2) = H1 * d2_ref - ratio * H2 * d1_ref
% pos = (H1 * d2_ref - ratio * H2 * d1_ref)/(H1 - ratio * H2)

% Calculate the new camera position to achieve the desired ratio
pos = (H1 * d2_ref - ratio * H2 * d1_ref) / (H1 - ratio * H2);

% Calculate the new focal length to maintain object A's size
% Using the equation: f_ref * H1 / d1_ref = f * H1 / (d1_ref - pos)
% Therefore: f = f_ref * (d1_ref - pos) / d1_ref
f = f_ref * (d1_ref - pos) / d1_ref;

end
