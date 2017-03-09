function updateObstaclePosition(conHan,t_ind)
% plotAnimation
% updates the phyiscal data for the constrain plot handle
%
% Input
%       conHan          The set of handles
%       t_ind           The time index to update the plot with
% (G)   constraints     The constraints structure
%
% Output
%       Updates to the constraint handles
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created 20160503 - BMorrell - for ASTRO_base
%

global constraints

% Turn off plotting
PlotOn = 0;

for i=1:length(constraints)
    if constraints(i).outside == 1 && all(constraints(i).rng==[1;2;3]) && constraints(i).moving
        %% Switch cases for different constraint types
        switch(constraints(i).fcnId)
            case 1
                %% Ellipsoid Constraint
                hold on
                [~,X,Y,Z] = EllipsoidFromShapeMatrix(constraints(i).x0(:,t_ind),constraints(i).A,[],[],[],PlotOn);
                set(conHan(i),'XData',X,'YData',Y,'ZData',Z)
            case 2
                %% Cylinder constriants
                hold on
                [x,y,z] = cylinder(constraints(i).r, 20);
                height = norm(constraints(i).a(:,t_ind));%norm(constraints(i).x2(:,t_ind)-constraints(i).x1(:,t_ind));
                fullSz = size(x);
                rowSz = [1, prod(fullSz)];
                xall = reshape(x,rowSz);
                yall = reshape(y,rowSz);
                zall = height*reshape(z,rowSz);
                % Recompute rotation matrix
                [Rot_Mat] = computeCylinderRotationMatrix(constraints(i).x1(:,t_ind),constraints(i).x2(:,t_ind));
                
                rotCoord = Rot_Mat' * [xall;yall;zall];
                X = reshape(rotCoord(1,:),fullSz)+constraints(i).x1(1,t_ind);
                Y = reshape(rotCoord(2,:),fullSz)+constraints(i).x1(2,t_ind);
                Z = reshape(rotCoord(3,:),fullSz)+constraints(i).x1(3,t_ind);
                
                % Change Data
                set(conHan(i),'XData',X,'YData',Y,'ZData',Z)
            case 3
                %% Cube Constraints
                if constraints(i).weight ~= 0
                    [~,vertices,faces] = plotCube(constraints(i).x0(:,t_ind),constraints(i),[],[],[],PlotOn);
                    
                    % Change Data
                    set(conHan(i),'Vertices',vertices,'Faces',faces);
                end
                
        end
    end
end
