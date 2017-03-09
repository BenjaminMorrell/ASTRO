function [maxCost,grad,maxIdx] = cylinderConstraintCost(states,constraint,in_out_scale,doGrad)
% cylinderConstraintCost
% Computes cost and cost gradient for an cylinder constraint. 
% 
% Input
%       states          The trajectory of the given planned path
%       constraint      Structure containing information on the constraint
%       in_out_scale    inside/outside multiplier. +1 (stay inside) or -1 (stay outside)
%       doGrad          Flag to set whether or not to compute the gradient
%
% Output
%       totalCost       The total cost
%       costGrad        The total cost gradient
%       maxUnsat        The cost of the maximum violation
%
% See documentation for details on the cost function equations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created 20160429 - BMorrell - for ASTRO_base

% initialise grad
grad = zeros(size(states,1),1);

% State postion
x = states(constraint.rng,:);

% Vectors
% Vector between end points
if constraint.moving == 0
    a = repmat(constraint.a,1,length(x(1,:))); %a*ones(1,length(x(1,:))); % Repeat in matrix
    x1_x = x-constraint.x1*ones(1,length(x(1,:))); %Vector from bottom to trajectory
    x2_x = x-constraint.x2*ones(1,length(x(1,:))); %Vector from top to test point
else
    a = constraint.a;           % Vector between end points on cylinder
    x1_x = x - constraint.x1;   % Vector from bottom to trajectory
    x2_x = x - constraint.x2;   % Vector from top to test point
end

% Determine which cost to apply for each part of the path
dot_bot = dot(a,x1_x);
dot_top = dot(-a,x2_x); % negative to reverse the direction of a so it comes from the same point as x2_x

x_endcap1 = x1_x(:,dot_bot<0);
x_endcap2 = x2_x(:,dot_top<0);
x_cylinder = x1_x(:,logical((dot_bot>=0).*(dot_top>=0)));

% Ellipsoid endcap costs
% if constraint.moving == 0
    % Bottom
    d_squared_mat = x_endcap1'*constraint.A*x_endcap1;
    costtmp_bot = in_out_scale*(d_squared_mat-ones(size(d_squared_mat)));
    costtmp_bot = diag(costtmp_bot);%Cost for each state position extracted
    % Top
    d_squared_mat = x_endcap2'*constraint.A*x_endcap2;
    costtmp_top = in_out_scale*(d_squared_mat-ones(size(d_squared_mat)));%.*eye(size(d_squared_mat));
    costtmp_top = diag(costtmp_top);%Cost for each state position extracted
% else
%     % Moving test case - make blank cases for the endcap costs
%     costtmp_bot = zeros(1,size(x_endcap1,2));
%     costtmp_top = zeros(1,size(x_endcap2,2));
% end
    
% Cylinder
if constraint.moving == 0
    a2 = repmat(constraint.a,1,length(x_cylinder(1,:))); %a*ones(1,length(x(1,:))); % Repeat in matrix
else
    % Moving case
    a2 = a(:,logical((dot_bot>=0).*(dot_top>=0))); %Select points of a that correspond with teh x_cylinder points (same logical input)
end
b = cross(a2,x_cylinder);%This gives |a||x1_x|sin(theta), for each set of points

%Distance to the line squared is |b|^2 / |a|^2, which leaves
% |x2_x|^2*sin^2(theta) = d^2
%Cost function is line d^2 - radius^2 (positive if outside)
if constraint.moving == 0
    costtmp_mid = in_out_scale.*(dot(b,b)./constraint.c - constraint.r^2*ones(1,length(b(1,:))));
else
    % Moving case - select c desired. 
    costtmp_mid = in_out_scale.*(dot(b,b)./constraint.c(:,logical((dot_bot>=0).*(dot_top>=0))) - constraint.r^2*ones(1,length(b(1,:))));
end

% Combine costs
% Initialise
costtmp = zeros(1,length(x1_x(1,:)));

% Add ellipsoid endcap costs
costtmp(dot_bot<0) = costtmp_bot;
costtmp(dot_top<0) = costtmp_top;

% Add cylinder cost
costtmp(logical((dot_bot>=0).*(dot_top>=0))) = costtmp_mid;

% Take the maximum violation
maxCost = max(costtmp);
% Id of maximum violation
maxIdx = find(costtmp==maxCost);
% a vector at max violation. 
max_a = a(:,maxIdx);

% Gradient computations (only for the maximum violation)
if doGrad && maxCost>0
    if dot(max_a,x1_x(:,maxIdx))<0 % bottom ellipsoid
        % Ellipsoid cost gradient
%         if constraint.moving == 0
            grad(constraint.rng) = in_out_scale*2*constraint.A*x1_x(:,maxIdx);
%         else
%             % If moving, gradient for endcaps is not used
%             grad(constraint.rng) = zeros(3,1);
%         end
    elseif dot(-max_a,x2_x(:,maxIdx))<0 % Top ellipsoid
        % Ellipsoid cost gradient
%         if constraint.moving == 0
            grad(constraint.rng) = in_out_scale*2*constraint.A*x2_x(:,maxIdx);
%         else
%             % If moving, gradient for endcaps is not used
%             grad(constraint.rng) = zeros(3,1);
%         end
    else % cylinder
        if constraint.moving == 0
            b = cross(max_a,x1_x(:,maxIdx));% |a||x1_x|sin(theta)
            % Derivative of the cost function explained above.
            grad(constraint.rng) = in_out_scale*2*cross(b,max_a)/constraint.c;
        else
            % Select vector 'a' and value 'c' corresponding to the worst violation time if moving
            b = cross(max_a,x1_x(:,maxIdx));% |a||x1_x|sin(theta)
            % Derivative of the cost function explained above.
            grad(constraint.rng) = in_out_scale*2*cross(b,max_a)/constraint.c(1,maxIdx);
        end
    end
end