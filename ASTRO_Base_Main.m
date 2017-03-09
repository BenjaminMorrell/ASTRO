% ASTRO - Admissible Subspace TRajectory Optimizer
%
clear all;close all
%% Initialise global variables
global OPT
global constraints

% initialise constraint structure
constraints = initConstraint;

%% Select test case to run 
% See README for explanation of different standard test cases
OPT.testcase      = 1;    % Test case general setup. Default: 1
OPT.trial         = 1;    % Trials for different test cases 1-7 now active

%% Load Settings
% Loads the default settings for the planner - look in this function for more details
LoadPlannerSettings();    % Loads into the OPT Structure
OPT.CONSTR.fcnType          = 3;        % Obstacle type to use

%% Load Boundary Conditions
% Setup new test cases and boundary conditions in this function
[BCs,NSPHEREs] = LoadBCs();

%% Basic Settings to modify
OPT.CONSTR.Weight_Ellipsoid = 1e0;      % Weighting for ellipsoid constraints
OPT.CONSTR.Weight_Cylinder  = 1e3;      % Weighting for Cylinder Constraints
OPT.CONSTR.Weight_Cube      = 1e3;      % Weighting for Cube Constraints

%% Load Constraints
[constr_count] = LoadConstraints();

%% Run Solver
% initial guess
guess = zeros(OPT.cfg.Nx*OPT.cfg.N,1);

% Solver
[CLegPoly,states,DataTrack] = ASTRO_solver(BCs,guess);

%% Plot Results
plotASTROResults([],states,DataTrack);
figure(21)
