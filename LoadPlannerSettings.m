function LoadPlannerSettings(varargin)
%> @file LoadPlannerSettings.m
% ======================================================================
%> @brief Autonmatically loads settings for the ASTRO Path Planner. And
%> loads the settings into the OPT global variable structure
%>

%> CREATED 20150621 - BMorrell - To Auto load all required settings
%> Input for different Defaults. Currently only Default == 1
%> MODIFIED 20160426 - BMorrell - Simplifying for ASTRO_Base

global OPT

if isempty(varargin)
    DefaultSet = 1;
else
    DefaultSet = varargin;
end

switch DefaultSet
    case 1
        % Set up Solver (generates Polynomials)
        SetUpSolver();
        
        % Acceleration Constraint
        OPT.performance   = 0;    % If want limit on acceleration
        OPT.CONSTR.accel_lim        = 5e-3;     % Acceleration limit
        OPT.CONSTR.Weight_accel     = 0.001;    %0.001        
        
        % Constraint Settings
        OPT.CONSTR.Weight_Ellipsoid = 1e3;      % Weighting for ellipsoid constraints
        OPT.CONSTR.Weight_Cylinder  = 1e3;      % Weighting for Cylinder Constraints
        OPT.CONSTR.Weight_Cube      = 100;      % Weighting for Cube Constraints
        OPT.CONSTR.Weight_Corridor  = 200;      % Corridor weighting
        OPT.CONSTR.Weight_Dynam     = 1e5;     % Weighting of dynamic constraints
                  
        %% Plotting
        OPT.PLOT.Obstacles       = 1; % To plot the constraints
        OPT.PLOT.Graphs          = 1; % To plot graphs tracking the optimisation
        OPT.PLOT.PathProgression = 1; % To plot the evolution of the paths
        OPT.PLOT.axis_limit      = 1; % limits for axis on plotting (as a cube)
        OPT.PLOT.manual          = 0; % Whether or not to have manual plotting
        OPT.PLOT.Animate         = 0; % Whether or not to animate
        
        %% Defaults for test cases
        OPT.static      = 1; % flag for using Static Obstacles
        OPT.performance = 0; % Flag for using performance constraints
        OPT.dynamic     = 0; % Flag to use dynamic constraints
        OPT.corridor    = 0; % Flag to have corridor constraints
end
