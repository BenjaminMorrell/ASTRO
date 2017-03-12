function plotASTROResults(CLegPoly,states,DataTrack)
% plotASTROResults
% Plots the path and constraints from the ASTRO run. 
% 
% Input
%       CLegPoly        The optimal polynomial coefficients 
%       states          The optimal trajectory 
%       DataTrack       A structure to store various data outputs
% (G)   constriant      Structure storing properties of the constraints 
%
% Output
%       Plots
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created 20160502 - BMorrell - for ASTRO_base

global OPT

% limits to the plot axis
axis_limit = OPT.PLOT.axis_limit;

%% Load Figure
figure(21+OPT.PLOT.j*100);
hold on

%% Plot Path
plot3(states(1,:),states(2,:),states(3,:),'k','linewidth',2);
plot3(states(1,1),states(2,1),states(3,1),'ko','linewidth',2);
plot3(states(1,end),states(2,end),states(3,end),'ko','linewidth',2);

%% Plot constriants
if OPT.PLOT.Obstacles
    plotConstraints();
end

%% Figure settings
axis equal;grid on;
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
set(gca,'FontSize',9,'FontName', 'Times')
title('Final Path')
axis([-axis_limit,axis_limit,-axis_limit,axis_limit,-axis_limit,axis_limit])
view(3)


%% Plot Graphs tracking optimisation performance
if OPT.PLOT.Graphs
    % Plot solver tracking graphs
    figure(22+OPT.PLOT.j*100)
    subplot(2,2,1)
    plot(DataTrack.totalCost)
    xlabel('iteration');ylabel('cost')
%     axis([0,size(DataTrack.totalCost,2),0,1000]);
    grid on
    subplot(2,2,2)
    plot(DataTrack.maxViol);
    xlabel('iteration');ylabel('maximum constraint violation')
    grid on
%     axis([0,size(DataTrack.totalCost,2),0,1000]);
    subplot(2,2,3)
    plot(DataTrack.Poly_C')
    xlabel('iteration');ylabel('Change in polynomial coefficients')
    grid on
    subplot(2,2,4)
    AX = plotyy(1:size(DataTrack.coeff,2),DataTrack.coeff,1:size(DataTrack.coeff,2),DataTrack.innerIter);
    ylabel(AX(1),'Backtracking Coefficient')
    ylabel(AX(2),'Line search iterations')
    xlabel('iterations');
    grid on
end

%% plot evolution of path 
if OPT.PLOT.PathProgression
    % Create figure
    figure(23+OPT.PLOT.j*100)
    hold on
    
    % define the colormap
    col_map = colormap(copper(size(DataTrack.Poly_C,2)));
    
    % plot end points
    plot3(states(1,1),states(2,1),states(3,1),'ko','linewidth',2);
    plot3(states(1,end),states(2,end),states(3,end),'ko','linewidth',2);

    % Plot each path
    for i = 1:size(DataTrack.Poly_C,2)
        % For each iteration of the polynomial, plot the path color change
        Plot_Col = col_map(i,:);
        
        % Get the states
        states_iteration = getTrajectory(DataTrack.Poly_C(:,i));
        
        % plot path
        plot3(states_iteration(1,:),states_iteration(2,:),states_iteration(3,:),'color',Plot_Col);

    end    
    
    % plot the obstacles
    if OPT.PLOT.Obstacles
        plotConstraints();
    end
    
    % Settings for the figure
    axis equal; grid on;
    xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
    set(gca,'FontSize',9,'FontName', 'Times')
    title('Path Evolutions through the optimisation')
    axis([-axis_limit,axis_limit,-axis_limit,axis_limit,-axis_limit,axis_limit])
    view(3)
end


if OPT.PLOT.Animate
    plotAnimation(states)
end

