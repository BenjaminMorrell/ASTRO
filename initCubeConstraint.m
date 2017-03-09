function constr_count = initCubeConstraint(x0,r,Rot_Mat,constr_count)
% initCubeConstraint
% Function takes the basic input to define an cube and generates the
% data strcuture to represent an ellipsoid constraint for ASTRO
% Input:
%       x0              centroid
%       r               Half the side length of the cube 
%       Rot_Mat         the rotation matrix to define the rotation of the
%                       cube
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
constraints(constr_count).fcnId     = 3; % cube constraint

%% Physical Properties
% Centroid
constraints(constr_count).x0        = x0;
% Radius
constraints(constr_count).r        = r;
% Rotation Matrix. Defined from Global to Body frames
constraints(constr_count).Rot_Mat   = Rot_Mat;

%% Constraint Properties
% To affect physical dimenions
constraints(constr_count).rng       = [1;2;3];
% A keep out constraint
constraints(constr_count).outside   = 1;
% Assign weighting
constraints(constr_count).weight    = OPT.CONSTR.Weight_Cube;