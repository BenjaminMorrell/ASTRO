function constr_count = initCylinderConstraint(x1, x2, r, l, constr_count)
% initCylinderConstraint() initializes a cylinder constraint
%   Creates a cylinder constraint with the following parameters
%       x1              Point defining bottom center of cylinder, or a matrix of the points at each point in time
%       x2              Point defining top center of cylinder, or a matrix of the points at each point in time
%       r               Radius of cylinder
%       l               Height of ellipsoidal endcap (semi-minor axis)
%       constr_count    count of the number of constriants
%      
% Output:
%       constraints     into the global constraints structure
%       constr_count    to count the number of constraints
% 
% Function computes the properties of an ellipsoid endcap to the cylinder
% and then inputs the properties into the constraints structure. Weighting
% of the constraints is set in OPT.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MODIFIED 20160427 - BMorrell - for ASTRO_Base
% MODIFIED 20161003 - BMorrell - Including dynamic cylinders without endcaps
global OPT
global constraints

constr_count = constr_count + 1;

if size(x1,2) > 1 % If the ellipsoid end point is defined as a set of points over time
    % Moving Cylinder case
    moving = 1;
else
    % Static case
    moving = 0;
end

%% Calculate parameters of ellipsoidal cap
a = x2-x1;          %Vector(s) between the end caps

if moving == 0
    [Rot_Mat] = computeCylinderRotationMatrix(x1,x2);
else
    % Moving case - end cap not used.
    Rot_Mat = eye(3);
    l = r; % Set the endcap to be a sphere so rotation does not matter
end
%% Initialise
constraints(constr_count)           = initConstraint;
% Constraint type
constraints(constr_count).fcnId     = 2; % Cylinder constraint

%% Physical Properties
% Endcap locations
constraints(constr_count).x1        = x1;
constraints(constr_count).x2        = x2;
% Cylinder Radius
constraints(constr_count).r         = r;
% Ellipsoid endcap Shape Matrix
constraints(constr_count).A         = Rot_Mat'*diag([1/r^2; 1/r^2; 1/l^2])*Rot_Mat;
% Rotation Matrix
constraints(constr_count).Rot_Mat   = Rot_Mat;
% Store vector and dot products for future use
constraints(constr_count).a         = a; % Vector between centers of ends
constraints(constr_count).c         = dot(a,a); % Distance between end caps squared

%% Constraint Properties
% To affect physical dimenions
constraints(constr_count).rng       = [1;2;3];
% A keep out constraint
constraints(constr_count).outside   = 1;
% Assign weighting
constraints(constr_count).weight    = OPT.CONSTR.Weight_Cylinder;

