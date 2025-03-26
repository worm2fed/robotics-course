function [endeff] = computeMiniForwardKinematics(rads1, rads2)

L1 = 1; % Shorter link
L2 = 2; % Longer link

% Compute elbow positions
x1 = L1 * cos(rads1);
y1 = L1 * sin(rads1);

x2 = L1 * cos(rads2);
y2 = L1 * sin(rads2);

% Solve for intersection of two circles centered at (x1, y1) and (x2, y2)
% with radius L2
d = sqrt((x2 - x1)^2 + (y2 - y1)^2); % Distance between elbow joints

if d > 2 * L2
    error('No valid solution: Elbows are too far apart.');
end

a = (L2^2 - L2^2 + d^2) / (2 * d);
h = sqrt(L2^2 - a^2);

% Midpoint between elbow joints
xm = (x1 + x2) / 2;
ym = (y1 + y2) / 2;

% Find end-effector position
ex = xm + h * (y2 - y1) / d;
ey = ym - h * (x2 - x1) / d;

endeff = [ex, ey];

end
