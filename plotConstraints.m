function hand = plotConstraints()
% plotConstraints
% Plots all the physical constraints for ASTRO. Designed to be plotted on
% an already created figure, and axis settings done externally. 
% 
% Input
%       constraints         The structure containing all the constraint details
%
% Output
%       hand                Handle for the constraint plots 
%       a plot of all constraints
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modified 20160502 - BMorrell - for ASTRO_base

global constraints

% initialise handles
hand(length(constraints)) = 1;
PlotOn = 1;         % flag to activate plots in functions

for i = 1:length(constraints)
    if all(constraints(i).rng==[1;2;3]) % If a physical constraint
        %% Set color if a keep in or keep out constaint
        if constraints(i).outside == 1
            % Keep out
            FaceColor = [0.5,0.5,0.5];
            EdgeColor = 'none';
            FaceAlpha = 0.5;
        else
            % Keep out
            FaceColor = [1,0,0];
            EdgeColor = 'none';
            FaceAlpha = 0.2;
        end
        
        %% Switch cases for different constraint types
        switch(constraints(i).fcnId)
            case 1 
                %% Ellipsoid Constraint
                hold on
                hand(i) = EllipsoidFromShapeMatrix(constraints(i).x0(:,end),constraints(i).A,FaceColor,EdgeColor,FaceAlpha,PlotOn);
                
            case 2
                %% Cylinder constriants
                hold on
                [x,y,z] = cylinder(constraints(i).r, 20);
                height = norm(constraints(i).x2(:,end)-constraints(i).x1(:,end));
                fullSz = size(x);
                rowSz = [1, prod(fullSz)];
                xall = reshape(x,rowSz);
                yall = reshape(y,rowSz);
                zall = height*reshape(z,rowSz);
                [Rot_Mat] = computeCylinderRotationMatrix(constraints(i).x1(:,end),constraints(i).x2(:,end));
                rotCoord = Rot_Mat' * [xall;yall;zall];
                hand(i) = surf(reshape(rotCoord(1,:),fullSz)+constraints(i).x1(1,end), ...
                    reshape(rotCoord(2,:),fullSz)+constraints(i).x1(2,end), ...
                    reshape(rotCoord(3,:),fullSz)+constraints(i).x1(3,end), ...
                    'FaceAlpha', FaceAlpha, 'EdgeColor', EdgeColor,...
                    'FaceColor',FaceColor);
            case 3
                %% Cube Constraints
                if constraints(i).weight ~= 0
                    hand(i) = plotCube(constraints(i).x0(:,end),constraints(i),FaceColor,EdgeColor,FaceAlpha,PlotOn);
                end
                
        end
    end
end