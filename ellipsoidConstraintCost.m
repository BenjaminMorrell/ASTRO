function [maxCost,grad,maxIdx] = ellipsoidConstraintCostInt(states,constraint,in_out_scale,doGrad)
% ellipsoidConstraintCost
% Computes cost and cost gradient for an ellipsoid constraint.
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
% Modified 20170309 - BMorrell - for testing different constraint options

global OPT

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

switch OPT.CONSTR.fcnType
    case 1
        % Original cost function
        % Calculate cost
        d_squared_mat = x'*constraint.A*x; % normalised "Distance squared" measure
        costtmp = in_out_scale*(d_squared_mat-ones(size(d_squared_mat))); % cost function (subtract 1)
        costtmp=diag(costtmp); % Take only the diagonal terms
        
        % Take the maximum violation
        maxCost = max(costtmp);
        % Id of maximum violation
        maxIdx = find(costtmp==maxCost);
        
        if numel(maxIdx)>1
            maxIdx = maxIdx(1);
        end
        
        % Compute gradient if option selected (only for the maximum violation)
        if doGrad && maxCost>0
            grad(constraint.rng) = in_out_scale*2*constraint.A*x(:,maxIdx);
        end
    case 2
        % Gaussian Cost function
        % Calculate cost
        d_squared_mat = diag(x'*constraint.A*x); % normalised "Distance squared" measure
        nSig = 2;
        coeff = 1/(sqrt(norm(constraint.A./nSig^2))*sqrt(2*pi));
        costtmp=coeff.*exp((-nSig^2/2).*d_squared_mat); % Take only the diagonal terms
        
        % Take the maximum violation
        maxCost = max(costtmp);
        % Id of maximum violation
        maxIdx = find(costtmp==maxCost);
        
        if numel(maxIdx)>1
            maxIdx = maxIdx(1);
        end
        
        % Compute gradient if option selected (only for the maximum violation)
        if doGrad && maxCost>0
            grad(constraint.rng) = -coeff.*nSig^2*constraint.A*x(:,maxIdx).*costtmp(maxIdx);
        end
        
    case 3
        % Inverse Quadratic r^2/d^2 - 1
        % Calculate cost
        dsq_scaled = diag(x'*constraint.A*x); % normalised "Distance squared" measure

        costtmp = 1./dsq_scaled - 1;
        
        % Take the maximum violation
        maxCost = max(costtmp);
        % Id of maximum violation
        maxIdx = find(costtmp==maxCost);
        
        if numel(maxIdx)>1
            maxIdx = maxIdx(1);
        end
        
        % Compute gradient if option selected (only for the maximum violation)
        if doGrad && maxCost>0
            grad(constraint.rng) = -2*constraint.A*x(:,maxIdx).*(costtmp(maxIdx)+1)^2;
        end
    case 4
        % Inverse distance r/d - 1
        % Calculate cost
        d_scaled = sqrt(diag(x'*constraint.A*x)); % normalised "Distance squared" measure

        costtmp = 1./d_scaled - 1;
        
        % Take the maximum violation
        maxCost = max(costtmp);
        % Id of maximum violation
        maxIdx = find(costtmp==maxCost);
        
        if numel(maxIdx)>1
            maxIdx = maxIdx(1);
        end
        
        % Compute gradient if option selected (only for the maximum violation)
        if doGrad && maxCost>0
            grad(constraint.rng) = -constraint.A*x(:,maxIdx).*(costtmp(maxIdx)+1)^3;
        end
        
    case 5
        % linear
        % Calculate cost
        d_scaled = sqrt(diag(x'*constraint.A*x)); % normalised "Distance squared" measure

        costtmp = 1 - d_scaled;
        
        % Take the maximum violation
        maxCost = max(costtmp);
        % Id of maximum violation
        maxIdx = find(costtmp==maxCost);
        
        if numel(maxIdx)>1
            maxIdx = maxIdx(1);
        end
        
        % Compute gradient if option selected (only for the maximum violation)
        if doGrad && maxCost>0
            grad(constraint.rng) = -constraint.A*x(:,maxIdx)./d_scaled(maxIdx);
        end
        
end
