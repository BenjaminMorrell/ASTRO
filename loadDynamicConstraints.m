function [constr_count] = loadDynamicConstraints(constr_count)
% Function loads the dynamic constraints for a range of trial cases
% Loads into the constraint global structure, using options (in global structure OPT)
% for testcase and trial. Input and output to track constraint count
% CREATED 20160427 - BMorrell - For ASTRO_Base

global OPT
global constraints

% Time vector
t_vec = linspace(0,OPT.tf,OPT.cfg.nSamp);

switch OPT.testcase 
    case 1
    switch OPT.dynamic
        case 1
            %% Spherical moving ellipsoid constraint only
            % Define Properties
            x_start = [-0.2;0;0]*ones(1,OPT.cfg.nSamp); % Starting position
            vel = [0.4/OPT.tf;0;0];                     % Velocity
            x0 = x_start + vel*t_vec;                   % Centroid over time
            r_sphere = 0.2;                             % Radius of sphere
            axes_sizes = [r_sphere;r_sphere;r_sphere];  % Axes vector
            Rot_Mat = eye(3);                           % Rotation matrix (identity for sphere)
  
            % Create Constraint (input into constraints global variable)
            [constr_count] = initEllipsoidConstraint(x0,axes_sizes,Rot_Mat,constr_count);
            
            % active moving option
            constraints(constr_count).moving = 1;   
        case 2
            %% Moving Cylinder constraint only
            % locations of the ends of the cylinder
            x1_s = [-0.4;0.05;0.2]*ones(1,OPT.cfg.nSamp);
            x2_s = [0.0;-0.05;-0.2]*ones(1,OPT.cfg.nSamp);
            vel1 = [0.6/OPT.tf;0;0]; 
            vel2 = [0.2/OPT.tf;0;0]; 
            x1 = x1_s + vel1*t_vec;                   % Centroid over time
            x2 = x2_s + vel2*t_vec;                   % Centroid over time
            % OR CAN DEFINE THESE TRAJECTORIES IN ANY OTHER WAY
            % Radius of Cylinder
            r = 0.15;
            % size of endcap (out from the circular end of the cylinder)
            l = r./100;
            
            % Create the cylinder constraint
            constr_count = initCylinderConstraint(x1, x2, r, l, constr_count);
            
            % active moving option
            constraints(constr_count).moving = 1;  
        case 3
            %% rotating cylinder constraint from a fixed point
            % locations of the ends of the cylinder
            x1 = [-0.5;0.0;-0.2]*ones(1,OPT.cfg.nSamp); % Location of rotation point
            
            % height of cylinder
            h = 1;
            
            % Rotating dynamics (in xz plane)
            theta = 3*pi/4;                           % Amount to rotate the cylidner
            theta_vec = linspace(0,theta,OPT.cfg.nSamp); % Vector for the angle over time
            % End point vector.
            x2(1,:) = x1(1,1) + h*sin(theta_vec);
            x2(3,:) = x1(3,1) + h*cos(theta_vec);
            % OR CAN DEFINE THESE TRAJECTORIES IN ANY OTHER WAY
            % Radius of Cylinder
            r = 0.2;
            
            % size of endcap (out from the circular end of the cylinder)
            l = r./100;
            
            % Create the cylinder constraint
            constr_count = initCylinderConstraint(x1, x2, r, l, constr_count);
            
            % active moving option
            constraints(constr_count).moving = 1;
            
            % Define Properties
            x_start = [-0.2;0;0]*ones(1,OPT.cfg.nSamp); % Starting position
            vel = [0.4/OPT.tf;0;0];                     % Velocity
            x0 = x_start + vel*t_vec;                   % Centroid over time
            r_sphere = 0.2;                             % Radius of sphere
            axes_sizes = [r_sphere;r_sphere;r_sphere];  % Axes vector
            Rot_Mat = eye(3);                           % Rotation matrix (identity for sphere)
            
            % Create Constraint (input into constraints global variable)
            [constr_count] = initEllipsoidConstraint(x0,axes_sizes,Rot_Mat,constr_count);
            
            % active moving option
            constraints(constr_count).moving = 1;
    end
    
end




