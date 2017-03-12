function cost = constraintCostRaw(CLegPoly,constraint)

% Get states
states = getTrajectory(CLegPoly);
doGrad = 1;

% Compute the cost, cost gradient and max violation ID
%     [c_cost,c_costGrad,maxIdx] = constraintCostGrad(states,constraint,doGrad);
[cost,~,~] = constraintCostGradInt(states,constraint,doGrad);

cost = cost^2;