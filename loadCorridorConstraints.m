function [constr_count] = loadCorridorConstraints(constr_count)
% Function loads the constraints to represent a keep in corridor for a range of trial cases
% Loads into the constraint global structure, using options (in global structure OPT)
% for testcase and trial. Input and output to track constraint count
% CREATED 20160427 - BMorrell - For ASTRO_Base

global OPT
global constraints

switch OPT.testcase 
    case 1
    switch OPT.corridor
        case 1
            %% Simple corridor
            % locations of the ends of the cylinder
            x1 = [0;-0.6;0];
            x2 = [0;0.6;0];
            % Radius of Cylinder
            r = 0.7;
            % size of endcap (out from the circular end of the cylinder)
            l = r.*1000;
            
            % Create the cylinder constraint
            constr_count = initCylinderConstraint(x1, x2, r, l, constr_count);
            
            % make it a keep-in constraint
            constraints(constr_count).outside = 0;            
        
    end
    
end