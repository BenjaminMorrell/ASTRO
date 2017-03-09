function [c_cost,c_costGrad,maxIdx] = constraintCostGrad(states,constraint,doGrad)
% constraintCostGrad
% Computes the constraint cost for ASTRO, with the input trajectory and the
% constraint.
%
% Input
%       states          The trajectory over time
%       constraint      The structure with the given constraint of interest
%       doGrad          Flag to run gradient or not
%
% Output
%       c_cost          The constriant cost
%       c_costGrad      The constraint cost gradient
%       maxID           The ID (in the time vector) for the maximum violation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created 20160429 - BMorrell - for ASTRO_base

global OPT

% Check if inside or outside
if constraint.outside
    in_out_scale = -1;
else
    in_out_scale = 1;
end

%% Compute Cost for different obstacle types
switch constraint.fcnId
    case 1
        %% Ellipsoid Constraint
        [maxCost,grad,maxIdx] = ellipsoidConstraintCost(states,constraint,in_out_scale,doGrad);

    case 2
        %% Cylinder Constraint
        [maxCost,grad,maxIdx] = cylinderConstraintCost(states,constraint,in_out_scale,doGrad);

    case 3
        %% Cube Constraint
        [maxCost,grad,maxIdx] = cubeConstraintCost(states,constraint,in_out_scale,doGrad);
                
end

% Cost zero if not violated
if all(maxCost<=0)
    % Constraint not violated
    maxIdx = 0;
    c_cost = 0;
else
    % Otherwise take one of the maximum costs (if multiple)
    c_cost = maxCost(1);
end

%% Compute gradient
% initialise with zero gradient
c_costGrad = zeros(OPT.cfg.Nx*OPT.cfg.N,1);

% If option selected and contraint violated, compute the gradient
if doGrad && maxIdx ~=0
    
    gradIdx = 1;
    
    % Run for each dimensions
    for i = 1:OPT.cfg.Nx
        %Form the sum of unscaled coefficients weighted by the gradient.  This
        %is equivalent to (df/dx)*(dX/dC) for the range of coefficients
        %corresponding to the current state variable
        base = grad(i)*OPT.cfg.P.posUnscaled(maxIdx,:)+....
            grad(i+OPT.cfg.Nx)*OPT.cfg.P.velUnscaled(maxIdx,:)+...
            grad(i+OPT.cfg.Nx*2)*OPT.cfg.P.accUnscaled(maxIdx,:);
        c_costGrad(gradIdx:gradIdx+OPT.cfg.N-1,1)= base(1,1:OPT.cfg.N)';
        gradIdx = gradIdx + OPT.cfg.N;
    end
    
    %Perform the final calculation 2*f*(df/Dx)' * dX/dC
    c_costGrad = 2*c_costGrad*c_cost;
    
end
