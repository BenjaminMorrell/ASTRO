function [constr_count] = initEllipsoidConstraint(x0, axes_sizes, Rot_Mat,constr_count)
% initEllipsoidConstraint
% Function takes the basic input to define an ellipsoid and generates the
% data strcuture to represent an ellipsoid constraint for ASTRO
% Input:
%       x0              centroid
%       axes_sizes      3x1 vector of the ellipsoid major, semi-major and minor
%                       axes. It is assumed that the vector is organised to align with the
%                       [x;y;z] axes before rotation
%       Rot_Mat         the rotation matrix to define the rotation of the ellipsoid
%                       from being aligned with the coordinate axes to its
%                       orientation. Defined from Global to Body frames
%       constr_count    the count for the number of constrants.
%
% Output:
%       constraint      The global data strcuture to conatin all the information to define
%                       the constraint
%       constr_count    updating the count of the number of constraints. 
%
% Note:
% the global OPT structure is used, and from it is where the obstacle weightings are
% set. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CREATED 20160427 - BMorrell
global OPT
global constraints

constr_count = constr_count + 1;

% Initialise
constraints(constr_count)           = initConstraint;
% Constraint type
constraints(constr_count).fcnId     = 1; % Generalised ellipsoid constraint

%% Physical Properties
% Centroid
constraints(constr_count).x0        = x0;
% Ellipsoid Shape Matrix
constraints(constr_count).A         = Rot_Mat'*diag([1/axes_sizes(1)^2; 1/axes_sizes(2)^2; 1/axes_sizes(3)^2])*Rot_Mat;
% Rotation Matrix. Defined from Global to Body frames
constraints(constr_count).Rot_Mat   = Rot_Mat;

%% Constraint Properties
% To affect physical dimenions
constraints(constr_count).rng       = [1;2;3];
% A keep out constraint
constraints(constr_count).outside   = 1;
% Assign weighting
constraints(constr_count).weight    = OPT.CONSTR.Weight_Ellipsoid;