function [c_cost,c_costGrad,maxIdx] = constraintCostGradInt(states,constraint,doGrad)
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
        [c_cost,grad] = ellipsoidConstraintCostInt(states,constraint,in_out_scale,doGrad);

    case 2
        %% Cylinder Constraint
        [c_cost,grad,maxIdx] = cylinderConstraintCost(states,constraint,in_out_scale,doGrad);

    case 3
        %% Cube Constraint
        [c_cost,grad,maxIdx] = cubeConstraintCost(states,constraint,in_out_scale,doGrad);
                
end
maxIdx = 1;
%% Compute gradient
% initialise with zero gradient
c_costGrad = zeros(OPT.cfg.Nx*OPT.cfg.N,1);

% If option selected and contraint violated, compute the gradient
if doGrad
    
    gradIdx = 1;
    
    % Run for each dimension
    for i = 1:OPT.cfg.Nx
        base = zeros(1,OPT.cfg.N);
        for j = 1:OPT.cfg.nSamp % For each point
            %Form the sum of unscaled coefficients weighted by the gradient.  This
            %is equivalent to (df/dx)*(dX/dC) for the range of coefficients
            %corresponding to the current state variable
            base = base + grad(i,j)*OPT.cfg.P.posScaled(j,:)+....
                grad(i+OPT.cfg.Nx,j)*OPT.cfg.P.velScaled(j,:)+...
                grad(i+OPT.cfg.Nx*2,j)*OPT.cfg.P.accScaled(j,:);
        end
            c_costGrad(gradIdx:gradIdx+OPT.cfg.N-1,1)= base(1,1:OPT.cfg.N)';
            gradIdx = gradIdx + OPT.cfg.N;
    end
    
    %Perform the final calculation 2*f*(df/Dx)' * dX/dC
    c_costGrad = 2*c_costGrad*c_cost;
    
end
