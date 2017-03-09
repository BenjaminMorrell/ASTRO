function varargout = plotCube(x0,constraint,FaceColor,EdgeColor,FaceAlpha,PlotOn)
% plotCube
% Plots a cube from the properties stored in constriant. For use with ASTRO
% constraints
% 
% Input
%       x0               The centroid at the given time. 
%       constriant       Structure storing properties of the cube 
%       FaceColor        Plot face color setting (R, G, B)
%       EdgeColor        Plot edge color setting
%       FaceAlpha        Face Alpha setting (how opaque)
%
% Output (in varargout)
%       g               Handle for the plot]
%       vertices        vertices of cube to manipulate elsewhere
%       faces           faces of cube to manipulate elsewhere
%       a plot
%
% Extensions could allow for rectangles and changes in
% orientation perhaps
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created 20160502 - BMorrell - for ASTRO_base

% define vertices
vertices = [0 0 0; 0 1 0; 1 1 0; 1 0 0; 0 0 1; 0 1 1; 1 1 1; 1 0 1];

% Adjust for a cube
vertices = (vertices - 0.5).*(2*constraint.r) + (x0*ones(1,8))'; % Reassign to centre and scale by r

% Rotate if applicable
if sum(sum(constraint.Rot_Mat))
    vertices = (constraint.Rot_Mat'*vertices')';% Apply transpose of Rot_Mat (going from body to global)
end

% Define faces - order of joining vertices
faces = [1,2,3,4;2,3,7,6;4,8,7,3;5,6,7,8;1,2,6,5;1,5,8,4];

% plot patch
if PlotOn
    g = patch('Vertices',vertices,'Faces',faces,...
        'FaceColor',FaceColor,'FaceAlpha',FaceAlpha,'EdgeColor',EdgeColor);
else
    g = 0;
end

% output
varargout{1} = g;
varargout{2} = vertices;
varargout{3} = faces;