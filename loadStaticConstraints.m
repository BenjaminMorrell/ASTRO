function [constr_count] = loadStaticConstraints(constr_count)
% Function loads the stationary constraints for a range of trial cases
% Loads into the constraint global structure, using options (in global structure OPT)
% for testcase and trial. Input and output to track constraint count
global OPT

switch OPT.testcase 
    case 1
        
        
    switch OPT.static
        case 1
            %% Spherical ellipsoid constraint only
            % Define Properties
            x0 = [0;0;0];                               % Centroid
            r_sphere = 0.2;                             % Radius of sphere
            axes_sizes = [r_sphere;r_sphere;r_sphere];  % Axes vector
            Rot_Mat = eye(3);                           % Rotation matrix (identity for sphere)
  
            % Create Constraint (input into constraints global variable)
            [constr_count] = initEllipsoidConstraint(x0,axes_sizes,Rot_Mat,constr_count);
            
        case 2
            %% Rotated ellipsoid constraint only
            % Define Properties
            x0 = [0;0;0];                           % Centroid
            axes_sizes = [0.2;0.6;0.1];             % Axes vector
            Rot_Mat = DCM_be([pi/4;2*pi/3;pi/2]);   % Rotation matrix
  
            % Create ellipsoid constraint (input into constraints global variable)
            [constr_count] = initEllipsoidConstraint(x0,axes_sizes,Rot_Mat,constr_count);
            
        case 3
            %% Cylinder constraint only
            % locations of the ends of the cylinder
            x1 = [-0.2;0.05;0.2];
            x2 = [0.2;-0.05;-0.2];
            % Radius of Cylinder
            r = 0.15;
            % size of endcap (out from the circular end of the cylinder)
            l = r./100;
            
            % Create the cylinder constraint
            constr_count = initCylinderConstraint(x1, x2, r, l, constr_count);
            
            
            % locations of the ends of the cylinder
            x1 = [0.2;0.05;0.2];
            x2 = [-0.2;-0.05;-0.2];
            % Radius of Cylinder
            r = 0.05;
            % size of endcap (out from the circular end of the cylinder)
            l = r./100;
            
            % Create the cylinder constraint
            constr_count = initCylinderConstraint(x1, x2, r, l, constr_count);
            
        case 4
            %% Cube Constraint only
            % Define Properties
            x0 = [0.0;0;0.0];                 % Centroid
            r = 0.1;                      % Half side length
            Rot_Mat = DCM_be([0.1;0.5;15*pi/180]);    % Rotation matrix
            
            % create the cube constraint
            constr_count = initCubeConstraint(x0,r,Rot_Mat,constr_count);
            
        case 5
            %% Cube and sphere
            % Define Properties
            x0 = [0;0;0];                 % Centroid
            r = 0.1;                      % Half side length
            Rot_Mat = DCM_be([0;0;0*pi/180]);    % Rotation matrix
            
            % create the cube constraint
            constr_count = initCubeConstraint(x0,r,Rot_Mat,constr_count);
            
        case 6
            
                       
    end
    
end
    
    
    

