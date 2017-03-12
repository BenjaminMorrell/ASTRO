function [totalCost,grad] = ellipsoidConstraintCostInt(states,constraint,in_out_scale,doGrad)
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
%       grad            The total cost gradient
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
        
        % total cost is the sum of costs
        totalCost = sum(costtmp);
        
        % Compute gradient if option selected (only for the maximum violation)
        if doGrad
            for i = 1:size(costtmp,1)
                grad(constraint.rng,i) = in_out_scale*2*constraint.A*x(:,i);
            end
        end
    case 2
        % Gaussian Cost function
        % Calculate cost
        d_squared_mat = diag(x'*constraint.A*x); % normalised "Distance squared" measure
        nSig = 3;
        coeff = 1/(sqrt(norm(constraint.A./nSig^2))*sqrt(2*pi));
        costtmp=coeff*exp((-nSig^2/2).*d_squared_mat); % Take only the diagonal terms
        
        % total cost is the sum of costs
        totalCost = sum(costtmp);

        
        % Compute gradient if option selected (only for the maximum violation)
        if doGrad
            for i = 1:size(costtmp,1)
                grad(constraint.rng,i) = -coeff*nSig^2*constraint.A*x(:,i).*costtmp(i);
            end
        end
        
    case 3
        % Inverse Quadratic r^2/d^2 - 1
        % Calculate cost
        dsq_scaled = diag(x'*constraint.A*x); % normalised "Distance squared" measure

        costtmp = 1./dsq_scaled - 1;
        
        % total cost is the sum of costs
        totalCost = sum(costtmp);
                
        % Compute gradient if option selected (only for the maximum violation)
        if doGrad
            for i = 1:length(costtmp)
                grad(constraint.rng,i) = -2*constraint.A*x(:,i).*(costtmp(i)+1)^2;
            end
        end
    case 4
        % Inverse distance r/d - 1
        % Calculate cost
        d_scaled = sqrt(diag(x'*constraint.A*x)); % normalised "Distance squared" measure

        costtmp = 1./d_scaled - 1;
        
        % total cost is the sum of costs
        totalCost = sum(costtmp);
                
        % Compute gradient if option selected
        if doGrad
            for i = 1:size(costtmp,1)
                grad(constraint.rng,i) = -constraint.A*x(:,i).*(costtmp(i)+1)^3;
            end
        end
        
    case 5
        % linear
        % Calculate cost
        d_scaled = sqrt(diag(x'*constraint.A*x)); % normalised "Distance squared" measure

        costtmp = 1 - d_scaled;
        
                % total cost is the sum of costs
        totalCost = sum(costtmp);
                
        % Compute gradient if option selected
        if doGrad
            for i = 1:size(costtmp,1)
                grad(constraint.rng,i) = -constraint.A*x(:,i)./d_scaled(i);
            end
        end
        
end
