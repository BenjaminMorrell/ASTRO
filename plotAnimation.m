function plotAnimation(states)
% plotAnimation
% Animates the trajectory, updating dynamic obstacles. 
% 
% Input
%       states          The trajectory
% (G)   constraints     The constraints structure
% (G)   OPT             The options structure
%
% Output
%       an animation plot
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Created 20160503 - BMorrell - for ASTRO_base

%% initialise
% Load Global OPT
global OPT

% Set the axis limit
axis_limit = OPT.PLOT.axis_limit;

% number of steps
nsteps = length(states(1,:));

% Time vector
t_vec = linspace(0,OPT.tf,OPT.cfg.nSamp);

% start figure
figure();
hold on

%% Plot complete path
plot3(states(1,1),states(2,1),states(3,1),'ko');
plot3(states(1,end),states(2,end),states(3,end),'bo');
plot3(states(1,:),states(2,:),states(3,:),'k','linewidth',1.2);

%% Plot dynamic parts
% Constraints
conHan = plotConstraints();

% Position
posHan = plot3(states(1,1),states(2,1),states(3,1),'r*','linewidth',2);

% Dynamic title
tHan=title('Animation of path progression t = 00.0s');

% Set figure properties
set(gca,'FontSize',9,'FontName','Times')
xlabel('x (m)');ylabel('y (m)');zlabel('z (m)');
axis equal;grid on;
view(3)
axis([-axis_limit,axis_limit,-axis_limit,axis_limit,-axis_limit,axis_limit])

%% Loop through for animation
for i=2:nsteps
    % Update Constraints positions
    updateObstaclePosition(conHan,i);
    
    % Update position
    set(posHan,'XData',states(1,i),'YData',states(2,i),'ZData',states(3,i))
    
    % manual - or autonomatic transition from frame to frame
    if ~OPT.PLOT.manual
        pause(0.05)
    else
        pause
    end
    
    % update title
    str = sprintf('Animation of path progression t = %4.1fs',t_vec(i));
    set(tHan,'String',str)
end

