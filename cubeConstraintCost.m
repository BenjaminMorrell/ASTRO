function [maxCost,grad,maxIdx] = cubeConstraintCost(states,constraint,in_out_scale,doGrad)
% cubeConstraintCost
% Computes cost and cost gradient for an cube constraint. 
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

% Check if constraint is moving
if constraint.moving
    x0 = constraint.x0;% Is a matrix if the object is moving
else
    x0 = constraint.x0*ones(1,length(states(1,:)));% Repeat centre if stationary
end

% Normalise state from centroid
x = (states(constraint.rng,:)-x0);% Difference to centre of constraint

% Rotate
if sum(sum(constraint.Rot_Mat))
    % States, x are in the global frame. Want to transform them into the
    % body frame to compare with the cube. 
    % Rotates to be in space where axes align with cube sides
    x = constraint.Rot_Mat*x; % Transform from global to body
end

% Cost for each dimension
x_cost = ((x(1,:).*x(1,:))./constraint.r^2-1);
y_cost = ((x(2,:).*x(2,:))./constraint.r^2-1);
z_cost = ((x(3,:).*x(3,:))./constraint.r^2-1);

% Check if there is a violation
cost_sign = ones(size(x_cost));
cost_sign(logical((x_cost>0)+(y_cost>0)+(z_cost>0))) = 0;

% Combined cost
costtmp = in_out_scale.*cost_sign.*(x_cost.*y_cost.*z_cost);

% Take the maximum violation
maxCost = max(costtmp);
% Id of maximum violation
maxIdx = find(costtmp==maxCost);

% Gradient computations (only for the maximum violation)
if doGrad && maxCost>0    
    % Derivative of the cost function
    grad(constraint.rng) = in_out_scale*(2*(1./(constraint.r^2))).*cost_sign(maxIdx).*[x(1,maxIdx)*y_cost(maxIdx)*z_cost(maxIdx);
                                                                    x(2,maxIdx)*x_cost(maxIdx)*z_cost(maxIdx);
                                                                    x(3,maxIdx)*x_cost(maxIdx)*y_cost(maxIdx)];
    % Rotate
    if sum(sum(constraint.Rot_Mat))
        % Gradient starts in the body frame. Need to transform it to the
        % global frame
        % Rotate back the gradient
        grad(constraint.rng) = constraint.Rot_Mat'*grad(constraint.rng); % From body to global
    end
end
