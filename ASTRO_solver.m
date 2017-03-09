function [CLegPoly,states,DataTrack] = ASTRO_solver(BCs,guess)
% ASTRO_solver()
% Function to run the optimisation for ASTRO to generate the optimal
% trajectory. Runs a Quasi-Newton, BFGS gradient descent optimisation to
% find the optimal trajectory that satisfies contraints.
%
% Input:
%       BCs             The boundary conditions for the trajectory
%       guess           the initial guess for the polynomial coefficients
% (G)   OPT             The settings in the global OPT structure
% (G)   constriants     The constriants stored in a global structure
%
% Output:
%       CLegPoly        The optimal polynomial coefficients 
%       states          The optimal trajectory output. 
%       DataTrack       A structure to store various data outputs
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MODIFIED 20160428 - BMorrell - For ASTRO_base

global OPT

%% Create Time Vector
% start timer
tic;
timeStart = toc;

%% Enforce Boundary Conditions with initial C set
% Create bounday value matrix
P_BC = [OPT.cfg.P.posScaled(1,:);OPT.cfg.P.velScaled(1,:);...
        OPT.cfg.P.posScaled(end,:);OPT.cfg.P.velScaled(end,:)];

% Initial, least squares guess between boundary conditions
bc = reshape(BCs,3,4)';
    
% intialise coefficients
C_start = zeros(OPT.cfg.Nx*OPT.cfg.N,1);

% Form initila coefficients
if norm(guess) ~= 0
    % If there is a non zero guess input
else
    for i = 1:OPT.cfg.Nx % For each dimension
        % Perform least squares on the 4 position and velocity conditions
        % Use only the first four coefficients 
        C_start((i-1)*OPT.cfg.N+1:(i-1)*OPT.cfg.N+4) = P_BC(:,1:4)\bc(:,i);
    end
end

% initial Coefficients
CLegPoly = C_start;

%% Get initial solution
% Trajectory from initial guess
states = getTrajectory(CLegPoly);

% Cost and gradient at initial guess
doGrad = 1;
[totalCost,costGrad,maxUnsat] = totalCostGrad(states,CLegPoly,doGrad);

% Start Data Track structure
DataTrack = initialiseDataTrack();
DataTrack.Poly_C(:,1) = CLegPoly;
DataTrack.totalCost = totalCost;
DataTrack.maxViol = maxUnsat;
DataTrack.gradient = costGrad;

%% Initialise parameter for optimisation loop
iter        = 0;                       % Interation counter 

exitflag    = 0;                       % set exit flag to zero
lastCost    = inf;                     % Initial "last cost" for first iteration
Hinv        = eye(length(CLegPoly));   % Initial Hessian Inverse
projMat = P_BC'*(P_BC*P_BC'\eye(4));   % Pre-compute re-used matrix for the P_BC enforcement

inerIterTrack = zeros(1,OPT.cfg.maxIter+1);% To track number of inner iterations


%% Optimisation Loop
while (~exitflag)
    %Take a step along the gradient descent direction (BFGS, quasi-Newton optimisation)
    newPoly = reshape(CLegPoly - OPT.cfg.stepSize*Hinv*costGrad,OPT.cfg.N,OPT.cfg.Nx);
    
    %Project the new coordinates onto the boundary condition constraint set.
    newPolySubMat = newPoly - (projMat*(P_BC*newPoly - bc));
    
    %Reshape back into a vector and subtract from original polynomial to get a feasible direction
    % delta_C
    polyStep = (reshape(newPolySubMat,OPT.cfg.Nx*OPT.cfg.N,1) - CLegPoly);
    
    % Change in cost from step (delta_C * del_F/del_C)
    firstOrderOpt = polyStep'*costGrad;
    
    % Check first order tolerance
    if abs(firstOrderOpt) < OPT.cfg.firstOrderTol
        % If decrease is below tolerance, exit the loop. 
        if OPT.cfg.trace
            display(['Exit: first order cost decrease in feasible direction ' num2str(firstOrderOpt) ' is less than exit tolerance in ' num2str(iter) ' iterations.']);
        end
        break;
    end
    
    %Check if new direction is a descent direction
    if firstOrderOpt > 0 % if cost is increasing
        %Reset the scaling to gradient descent
        Hinv = eye(length(CLegPoly));
        warning('Resetting Hessian to Identity');
    end
       
    %% Armijo Update - line-search
    % initialise for Armijo update
    coeff       = 1;    % Starting step size
    exit        = 0;    % Exit flag
    innerIter   = 0;    % Iteration count
    
    while (~exit)
        % Evalulate new polynomial and cost
        polyTgt = CLegPoly+coeff*polyStep;       % Apply step
        newStates = getTrajectory(polyTgt); % Get trajectory from the step
        % Costs of the new polynomial
        [newCost,~,~] = totalCostGrad(newStates,polyTgt,0);
        
        %Check to see if cost reduction is large enough to continue
        if ((totalCost - newCost) >= -coeff*OPT.cfg.sigma*costGrad'*polyStep)
            % If it satisfies, then exit
            break;
        end
        
        % Check upper limit for iterations
        if innerIter > OPT.cfg.maxArmijo
            warning('Large Armijo Update')
            break;
        end
        
        % Step iteration counter
        innerIter = innerIter + 1;
        
        % Backtrack the step length
        coeff = coeff*OPT.cfg.beta;
        
    end
    % Store the inner iterations
    inerIterTrack(iter+1) = innerIter;
    
    % Update with the cost-reduced polynomial
    CLegPoly = polyTgt;
    
    %% BFGS Hessian Update 
    %1) Calculate difference in gradients (refer to documentation for details)
    % New cost and gradients
    [newCost,newGrad,maxViol] = totalCostGrad(newStates,CLegPoly,1);
    
    % Change in gradient
    y = reshape(newGrad - costGrad,OPT.cfg.N,OPT.cfg.Nx);
    % Project into feasible direction
    y= y - projMat*P_BC*y;
    y = reshape(y,OPT.cfg.N*OPT.cfg.Nx,1);
    
    %2) Get size of step
    sk = coeff*polyStep;
    
    %3) Update the Hessian Inverse
    y_s_prod = y'*sk; % pre-compute for speed
    
    %Check for positive curvature (to ensure pos def Hinv)  
    if y_s_prod <= 0
        %If curvature is not positive, reset the hessian
        Hinv = eye(length(CLegPoly)); 
    else
        % BFGS update of hessian inverse. See http://terminus.sdsu.edu/SDSU/Math693a_f2013/Lectures/18/lecture.pdf.
        % and simplicifcation of update for computational efficiency (shown https://en.wikipedia.org/wiki/Broyden%E2%80%93Fletcher%E2%80%93Goldfarb%E2%80%93Shanno_algorithm)
        Hinv = Hinv + (y_s_prod + y'*Hinv*y)*(sk*sk')/(y_s_prod)^2 - (Hinv*y*sk' + sk*y'*Hinv)/(y_s_prod);
    end
    
    
    %% Check tolerances for convergence
    DataTrack.costDiff(1,i) = newCost-lastCost;
    if (abs(newCost-lastCost)/abs(lastCost) < OPT.cfg.exitTol) && all(maxViol <= 1e-3)
        % Exit if the cost change is below tolerance and the maximum
        % constraint violation cost is sufficiently small
        if OPT.cfg.trace
            display(['Exit: cost difference is less than exit tolerance in ' num2str(iter) ' iterations.']);
        end
        break;
    end
    % Set the last cost
    lastCost = newCost;
    
    %Initialize next loop
    totalCost   = newCost;  
    costGrad    = newGrad;
    iter        = iter+1;   % Step iterations of outer loop
    
    % Check against iteration limit - exit if over
    if iter > OPT.cfg.maxIter
        warning('Exit: iteration limit exceeded');
        break;
    end
    
    % Check the timing of the loop
    current_time = toc;
    if current_time - timeStart > OPT.cfg.timeLim
        % Exit if time limit exceeded
        warning('Exit: time limit exceeded');
        break;
    end    
    
    %% Store Data
    DataTrack.Poly_C(:,iter+1)      = CLegPoly;
    DataTrack.polystep(:,iter)      = polyStep;
    DataTrack.totalCost(1,iter+1)   = totalCost;
    DataTrack.maxViol(1,iter+1)     = maxViol;
    DataTrack.gradient(:,iter+1)    = costGrad;
    DataTrack.coeff(1,iter)         = coeff;
    DataTrack.innerIter(1,iter)     = innerIter;
end

%% Store Data at end
DataTrack.Poly_C(:,iter+1)      = CLegPoly;
DataTrack.polystep(:,iter)      = polyStep;
DataTrack.totalCost(1,iter+1)   = totalCost;
DataTrack.maxViol(1,iter+1)     = maxViol;
DataTrack.gradient(:,iter+1)    = costGrad;
DataTrack.coeff(1,iter)         = coeff;
DataTrack.innerIter(1,iter)     = innerIter;

% output trajectory
states = newStates;