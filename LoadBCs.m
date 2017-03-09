function [BCs,NSPHEREs] = LoadBCs()
% Function loads the boundary counditions for each SPHERE in a given test
% case.
% BCs in format for each column, a different SPHERE, as
% [X0;X_dot0;Xf;X_dotf]

% MODIFIED - 20160426 - BMorrell - simplify for ASTRO_Base

global OPT

initial_offset = 1e-5; %to take away from any symmetry saddle points
BCs = []; % initialise

% final time for trajectory
OPT.tf = 100; 

switch OPT.testcase
    case 1
        % Simple Initial Static Test Case
        % initial Position
        X0 = [0.1,-0.5,0,0,0,0]-initial_offset*[1,0,0, zeros(1,3)];
        % Final Position
        Xf = [0,0.5,0,0,0,0];
        % Store BCs
        BCs(:,1) = [X0';Xf'];
        
        % Number of SPHERES
        NSPHEREs = 1;
        
                % Set options
        OPT.static      = 1; % Static Obstacle Options
        OPT.performance = 0; % Flag for using performance constraints
        OPT.dynamic     = 0; % Dynamic Obstacle Options
        OPT.corridor    = 0; % Corridor Options
        
        switch OPT.trial
            case 1 % Spherical object only
                OPT.static          = 1; % Static Obstacle Options
            case 2 % Elliptical constraint
                OPT.static          = 2; % Static Obstacle Option
            case 3 % Corridor Constraint
                OPT.static          = 3; % Static Obstacle Option
            case 4 % Cube Constraint
                OPT.static          = 4; % Static Obstacle Option
            case 5 % Dynamic Constraint
                OPT.static          = 0; % flag for using Static Obstacles
                OPT.dynamic         = 1; % Dynamic Obstacles option
                OPT.PLOT.manual     = 0; % to manual progress plotting
                OPT.PLOT.Animate    = 1; % to plot an animation
            case 6 % Corridor and sphere
                OPT.static          = 1; % Static Obstacle Options
                OPT.corridor        = 1; % Corridor Option
            case 7
                OPT.static          = 5; % Cube testing
            case 8
                % Moving Cylinder
                OPT.static          = 0; % flag for using Static Obstacles
                OPT.dynamic         = 2; % Dynamic cylinder testing
                OPT.PLOT.manual     = 0; % to manual progress plotting
                OPT.PLOT.Animate    = 1; % to plot an animation
            case 9
                % only rotating cylinder
                OPT.static          = 0; % flag for using Static Obstacles
                OPT.dynamic         = 3; % Dynamic cylinder testing
                OPT.PLOT.manual     = 0; % to manual progress plotting
                OPT.PLOT.Animate    = 1; % to plot an animation
        end
        
        % Plot options
        OPT.PLOT.axis_limit = 1;

    case 99
        % Customise this testcase for your own tests
        % initial Position
        X0 = [0 -0.5 0, 0 0 0]-initial_offset*[ones(1,3), zeros(1,3)];
        % Final Position
        Xf = [0.0 0.5 0.0, 0 0 0];
        % Store BCs
        BCs(:,1) = [X0';Xf'];
        
        % Number of SPHERES
        NSPHEREs = 1;       
end


%% Scale legendre polynomials
OPT.cfg.P.posScaled = OPT.cfg.P.posUnscaled*(OPT.tf/2);
OPT.cfg.P.velScaled = OPT.cfg.P.velUnscaled; % Legendre Polynomials represent velocity
OPT.cfg.P.accScaled = OPT.cfg.P.accUnscaled*2/OPT.tf;
