function [elbow,endeff] = computeRrForwardKinematics(rads1,rads2)
%%GIVEN THE ANGLES OF THE MOTORS, return an array of arrays for the
%%position of the elbow [x1,y1], and endeffector [x2,y2]
    L1 = 1; % Given length
    L2 = 1; % Given length

    % Compute elbow position
    x1 = L1 * cos(rads1);
    y1 = L1 * sin(rads1);
    elbow = [x1, y1];

    % Compute end effector position
    x2 = x1 + L2 * cos(rads1 + rads2);
    y2 = y1 + L2 * sin(rads1 + rads2);
    endeff = [x2, y2];
end
