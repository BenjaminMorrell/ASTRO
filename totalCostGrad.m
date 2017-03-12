function [totalCost,costGrad,maxUnsat] = totalCostGrad(states,CLegPoly,doGrad)
% totalCostGrad
% Computes the augmented cost for ASTRO, with the input polynomail coefficients
% and the legendre polynomial integrals stored in the global OPT structure.
% Includes cost for the path and the constraint violation. 
% 
% Input
%       states          The trajectory of the given planned path
%       CLegPoly        The polynomial coefficients
%       doGrad          Flag to set whether or not to compute the gradient
%       OPT             The global options and settings structure
%
% Output
%       totalCost       The total cost
%       costGrad        The total cost gradient
%       maxUnsat        The cost of the maximum violation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created 20160429 - BMorrell - for ASTRO_base

global constraints

%% Path cost and gradient - optimisation cost function
[path_cost, path_costGrad] = pathCostGrad(CLegPoly, doGrad);

%% initialise total cost
totalCost = path_cost; % start with just the path cost
costGrad = path_costGrad;
maxUnsat = 0; % start with no constraint violation 

%% Loop for each constraint and augment the cost and gradient
for i = 1:length(constraints)
    % Compute the cost, cost gradient and max violation ID
%     [c_cost,c_costGrad,maxIdx] = constraintCostGrad(states,constraints(i),doGrad);
    [c_cost,c_costGrad,maxIdx] = constraintCostGradInt(states,constraints(i),doGrad);
    
    % Test numerical gradient
%     fun = @(X) constraintCostRaw(X,constraints(i));
%     J = JacobGen(fun,CLegPoly,[])';
    
    %Check for an active constraint
    if all(maxIdx>0)
        % Add cost of constraint to total cost, weighted (note that the cost function is c_cost^2)
        totalCost = totalCost + constraints(i).weight*c_cost^2;
        
        % If computing gradient
        if doGrad 
            % Add constraint gradient to cost gradient
            costGrad = costGrad + constraints(i).weight*c_costGrad;
        end
        
        % Update maxUnsat - the maximum constraint violation
        maxUnsat = max(maxUnsat,constraints(i).weight*c_cost^2);
    end
end