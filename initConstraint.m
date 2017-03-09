function constraint = initConstraint()
% Initialises the constraint structure for ASTRO
% MODIFIED 20160426 - BMorrell for ASTRO_base

constraint = struct('fcnId', 0, ... Type of constraint
           'x0',zeros(3,1),... Centroid - becomes a matrix for moving obstacles
           'A',zeros(3,3),... Shape Matrix (for Ellipsoids)
           'rng',zeros(3,1),... What dimenions it affects
           'outside',1,... % Flag to indicate if it is a keep in or keep out constraint
           'weight',1,... % Initial weighting for cost function
           'x1', zeros(3,1),... % Used for Cylinder endcap centre and used to define the x, y and z distances for a rectangular prism
           'x2', zeros(3,1),...
           'r', 0,... % Radius of constraint (used for Cylinders and Cubes)
           'Rot_Mat', zeros(3),... % Rotation Matrix (used for skewed constraints). Defined from Global to Body frames
           'moving', 0 ,... % Flag to indicate if it is a moving constraint
           'a',   zeros(3,1),... % Vector between endcaps for cylinder
           'c',   zeros(3,1)... % Dot product of a for cylinder
           );

       