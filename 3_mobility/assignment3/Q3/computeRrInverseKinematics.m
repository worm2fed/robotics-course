function [rads1, rads2] = computeRrInverseKinematics(X, Y)

L1 = 1;
L2 = 1;

% Compute theta2
c2 = (X^2 + Y^2 - L1^2 - L2^2) / (2 * L1 * L2);
rads2 = acos(c2);

% Compute theta1
rads1 = atan2(Y, X) - atan2(L2 * sin(rads2), L1 + L2 * cos(rads2));

end
