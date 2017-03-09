function cfg = initAstro()
%INITASTRO Initialize the solver configuration

cfg.N             = 12;             %Legendre Polynomial Order
cfg.Nx            = 3;              %Number of states
cfg.exitTol       = 1e-8;           %Exit if relative difference in cost between steps is less than this value
cfg.firstOrderTol = 1e-6;           %Exit if cost decrease in feasible direction is less than this
cfg.projGrad      = 1;              %Set to 1 to use projected gradient
cfg.stepSize      = 1;              %Initial size of step taken in descent
cfg.maxIter       = 200;            %Maximum number of iterations before exiting
cfg.nSamp         = 200;            %number of trajectory points to evaluate constraints at
cfg.trace         = 1;              % To output tracking printout for the optimisation
cfg.timeLim       = 100;             %Exit if the time for the solver exceeds this

%Line search option. 1 - Wolfe Strategy, 2 - Armijo update, 0 - alpha = 1                                                                    
cfg.LineSearch    = 2;               
cfg.maxArmijo     = 1000;           %Maximum Armijo Rule iterations                                    
cfg.maxZoom       = 10;             %Maximum "Zoom" iterations   

% This term determines the step size reduction at each iteration of the
% Armijo Rule update.  At each inner iteration of the line search
% stepSize(k) = beta*stepSize(k-1)
cfg.beta          = 0.85;        

% This term attempts to ensure a "sufficient decrease" condition.  When
% choosing a step size, the cost function descrease must be greater than 
% 'sigma' times the first order approximation of the cost function decrease
% with the same step size.
%   (cost - newCost) >= -sigma*costGradient'*currentStep
% Typically this number is kept quite small.
cfg.sigma         = 1e-8;           % Armijo condition

end

