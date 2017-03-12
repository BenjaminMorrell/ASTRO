% ASTRO - Admissible Subspace TRajectory Optimizer
%
clear all;close all
%% Initialise global variables
global OPT
global constraints
global TIMER
TIMER.constrCalc = 0;

% initialise constraint structure
constraints = initConstraint;

%% Select test case to run 
% See README for explanation of different standard test cases
OPT.testcase      = 1;    % Test case general setup. Default: 1
OPT.trial         = 10;    % Trials for different test cases 1-7 now active

%% Load Settings
% Loads the default settings for the planner - look in this function for more details
LoadPlannerSettings();    % Loads into the OPT Structure
OPT.CONSTR.fcnType          = 2;        % Obstacle type to use

%% Load Boundary Conditions
% Setup new test cases and boundary conditions in this function
[BCs,NSPHEREs] = LoadBCs();

%% Basic Settings to modify
OPT.CONSTR.Weight_Ellipsoid = 1e-2;      % Weighting for ellipsoid constraints
OPT.CONSTR.Weight_Cylinder  = 1e3;      % Weighting for Cylinder Constraints
OPT.CONSTR.Weight_Cube      = 1e3;      % Weighting for Cube Constraints

%% Load Constraints
[constr_count] = LoadConstraints();

%% Loop through constriant types
nFcn = 2;

for j = 1:nFcn
    if j ==1
        % Old constraint type
        OPT.CONSTR.fcnType          = 1;        % Obstacle type to use
        OPT.CONSTR.fcnInt           = 0;        % Max point constraint if = 0
        OPT.CONSTR.Weight_Ellipsoid = 1e-2;      % Weighting for ellipsoid constraints
    elseif j ==2
        % Guassian integral constraint type
        OPT.CONSTR.fcnType          = 2;        % Obstacle type to use
        OPT.CONSTR.fcnInt           = 1;        % Addition constraint if = 1
        OPT.CONSTR.Weight_Ellipsoid = 1e-2;      % Weighting for ellipsoid constraints
    end
%% Iterate on the boundary conditions
nTests = 100;
rng(3);
meanX0 = [0;-1.2;0;0;0;0];
sdX = [0.5;0.25;0.5;0;0;0];
meanXf = [0; 1.2;0;0;0;0];

for i = 1:nTests
    TIMER.constrCalc = 0;
    
    % Generate new BCs
    % initial Position
    X0 = meanX0 + sdX.*randn(6,1);
    % Final Position
    Xf = meanXf + sdX.*randn(6,1);
    % Store BCs
    BCs(:,1) = [X0;Xf];
    
    %% Run Solver
    % initial guess
    guess = zeros(OPT.cfg.Nx*OPT.cfg.N,1);
    
    % timing
    t1 = tic;
    
    % Solver
    [CLegPoly,states,DataTrack] = ASTRO_solver(BCs,guess);
    
    % Tracking timing
    timer = toc(t1);
    
    % compute path cost
    [path_cost, ~] = pathCostGrad(CLegPoly, 0);
    
    %% Capture Data
    ConstrData.iter(j,i)          = size(DataTrack.Poly_C,2)-1;
    ConstrData.totalTime(j,i)     = timer;
    ConstrData.constrTime(j,i)    = TIMER.constrCalc;
    ConstrData.totalCost(j,i)     = DataTrack.totalCost(end);
    ConstrData.pathCost(j,i)      = path_cost;
    ConstrData.constrViol(j,i)    = DataTrack.maxViol(end);
    
    %% Plot Results
    OPT.PLOT.j = j;OPT.PLOT.axis_limit = 2;
    plotASTROResults([],states,DataTrack);
    figure(21+OPT.PLOT.j*100)
    
end
end

%% Plot results
figure(1)
subplot(4,1,1)
plot(ConstrData.iter')
grid on; xlabel('test'); ylabel('iterations')
legend('Fcn1','Fcn2')

subplot(4,1,2)
plot(ConstrData.totalTime')
% hold on
% plot(ConstrData.constrTime')
grid on; xlabel('test'); ylabel('Comp Time')
legend('Fcn1 total','Fcn2 total');%,'Fcn1 constr','Fcn2 constr')

subplot(4,1,3)
% plot(ConstrData.totalCost')
% hold on
plot(ConstrData.pathCost')
grid on; xlabel('test'); ylabel('Cost')
legend('Fcn1 path','Fcn2 path')%'Fcn1 total','Fcn2 total',

subplot(4,1,4)
plot(ConstrData.constrViol')
grid on; xlabel('test'); ylabel('constraint violation')
legend('Fcn1','Fcn2')


