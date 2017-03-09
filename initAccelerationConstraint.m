function [constr_count] = initAccelerationConstraint(constr_count)
% initAccelerationConstraint
% Defines a constraint to restrict the acceleration of the trajectory
% Input:
%       constr_count    the count for the number of constrants.
%
% Output:
%       constraint      The global data strcuture to conatin all the information to define
%                       the constraint
%       constr_count    updating the count of the number of constraints. 
%
% Note:
% the global OPT structure is used, and from it is where the obstacle
% weightings and the limits are set. 
% Constraint is a spherical "keep in" constraint operating on accelration
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MODIFIED 20160427 - BMorrell - for ASTRO_base
global OPT
global constraints

% step counter
constr_count = constr_count+1;

OPT.CONSTR.accel_lim,OPT.CONSTR.Weight_accel

% Initialise
constraints(constr_count)           = initConstraint;
% Constraint type
constraints(constr_count).fcnId     = 1; % ellipsoid constraint

%% Properties
% Centroid
constraints(constr_count).x0        = [0;0;0]; % Centre at 0 acceleration
% Ellipsoid Shape Matrix - spherical
constraints(constr_count).A         = eye(3)/(OPT.CONSTR.accel_lim)^2;

%% Constraint Properties
% To affect acceleration dimenions
constraints(constr_count).rng       = [7;8;9];
% A keep in constraint
constraints(constr_count).outside   = 0;
% Assign weighting
constraints(constr_count).weight    = OPT.CONSTR.Weight_accel;