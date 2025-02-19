function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

axes = zeros(6,2);

% Get edges for triangle 1 and all potential separating axes
edge1_t1 = P1(2,:) - P1(1,:);
axes(1,:) = [-edge1_t1(2), edge1_t1(1)];
edge2_t1 = P1(3,:) - P1(2,:);
axes(2,:) = [-edge2_t1(2), edge2_t1(1)];
edge3_t1 = P1(1,:) - P1(3,:);
axes(3,:) = [-edge3_t1(2), edge3_t1(1)];

% Get edges for triangle 2 and all potential separating axes
edge1_t2 = P2(2,:) - P2(1,:);
axes(4,:) = [-edge1_t2(2), edge1_t2(1)];
edge2_t2 = P2(3,:) - P2(2,:);
axes(5,:) = [-edge2_t2(2), edge2_t2(1)];
edge3_t2 = P2(1,:) - P2(3,:);
axes(6,:) = [-edge3_t2(2), edge3_t2(1)];

% Test each axis
for i = 1:6
    axis = axes(i,:);

    % Project triangle 1 onto axis
    proj1_1 = dot(P1(1,:), axis);
    proj1_2 = dot(P1(2,:), axis);
    proj1_3 = dot(P1(3,:), axis);
    min1 = min([proj1_1, proj1_2, proj1_3]);
    max1 = max([proj1_1, proj1_2, proj1_3]);

    % Project triangle 2 onto axis
    proj2_1 = dot(P2(1,:), axis);
    proj2_2 = dot(P2(2,:), axis);
    proj2_3 = dot(P2(3,:), axis);
    min2 = min([proj2_1, proj2_2, proj2_3]);
    max2 = max([proj2_1, proj2_2, proj2_3]);

    % Check for separation
    if max1 < min2 || max2 < min1
        flag = false;
        return;
    end
end

% If we get here, no separating axis was found
flag = true;

% *******************************************************************
end
