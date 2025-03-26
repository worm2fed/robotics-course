function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate
%
% NOTE: Once the route array has been updated, pass the array as is instead of appending with zeros or NAN

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
% Initialize route with start position
route = start_coords;
current_pos = start_coords;
num_its = 0;

% Continue until reaching goal or max iterations
while (norm(current_pos - end_coords) >= 2.0 && num_its < max_its)
    % Get gradient at current position (using round since we need integer indices)
    current_x = round(current_pos(1));
    current_y = round(current_pos(2));

    % Ensure we stay within array bounds
    current_x = min(max(current_x, 1), size(gx, 2));
    current_y = min(max(current_y, 1), size(gx, 1));

    % Get gradient values at current position
    grad_x = gx(current_y, current_x);
    grad_y = gy(current_y, current_x);

    % Normalize gradient
    grad_magnitude = sqrt(grad_x^2 + grad_y^2);
    if grad_magnitude > 0
        grad_x = grad_x / grad_magnitude;
        grad_y = grad_y / grad_magnitude;
    end

    % Update position (step size = 1.0)
    new_pos = current_pos + [grad_x, grad_y];

    % Add new position to route
    route(end + 1, :) = new_pos;

    % Update current position
    current_pos = new_pos;

    % Increment iteration counter
    num_its = num_its + 1;
end

% *******************************************************************
end
