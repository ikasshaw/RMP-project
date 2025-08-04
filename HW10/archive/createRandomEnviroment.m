%% createRandomEnviroment
% This function generates a random convex boundary and randomly space
% convex configuration space obstacles (C-obstacles). For the final
% project, you will need to generate C-obstacles from a robot geometry and
% obstacle geometry. An adaptation of this code can be used.
%
%   Usage notes:
%       (1) Your "plotCObstacle.m" function must be in the MATLAB path for
%           this code to show obstacles.
%       (2) Your "plotCObstacle.m" function should have the following
%           syntax:
%               ptc = plotCObstacle(CB{i},i)
%
%   M. Kutzer, 22Jul2024, JHU-EP

%% User defined parameters
% Boundary
maxBndsSides = 8;   % user-defined max sides for obstacle
xBnds = [-100,100]; % user-defined approximate boundary x-limits
yBnds = [-100,100]; % user-defined approximate boundary y-limits

% Obstacles
maxObsSides = 14; % user-defined max sides per obstacle
nObs = 10;        % user-defined total obstacles

%% Create CB Obstacle figure & axes
% Figure and axes setup
fig = figure('Name','Random Obstacles','NumberTitle','off');
axs = axes('Parent',fig,'NextPlot','add','DataAspectRatio',[1 1 1]);
axis(axs,'tight');

%% Create bounds
% Generate random vertices
bounds = ...
    [diff(xBnds); diff(yBnds)].*rand(2,maxBndsSides)+[xBnds(1); yBnds(1)];

% Get indices for convex obstacle
idx = convhull(bounds(1,:),bounds(2,:));
% Remove wrap around
idx(end) = [];
% Reorder vertices in CCW direction
bounds = bounds(:,idx);

% Plot bounds
psBnds = polyshape(bounds(1,:),bounds(2,:));
pltBnds = plot(axs,psBnds,'FaceColor','k','EdgeColor','k','FaceAlpha',0.1);

%% Create random CB Obstacles
% Number of CB Obstacles


% Define obstacle limits
xx = [min(bounds(1,:)),max(bounds(1,:))];
yy = [min(bounds(2,:)),max(bounds(2,:))];

% CB Obstacle Generation
i=1;
while true

    % Generate random vertices
    CB{i} = ...
        (1.4/nObs)*[diff(xx); diff(yy)].*rand(2,maxObsSides) + ...
        [xx(1); yy(1)] + [diff(xx); diff(yy)].*rand(2,1);

    % Get indices for convex obstacle
    idx = convhull(CB{i}(1,:),CB{i}(2,:));
    % Remove wrap around
    idx(end) = [];
    % Reorder vertices in CCW direction
    CB{i} = CB{i}(:,idx);

    % Check if obstacle is within the boundary
    tfIn = isinterior(psBnds,CB{i}(1,:),CB{i}(2,:)); 
    if any(~tfIn)
        % Polygon is not fully contained within the boundary
        CB(i)= [];
        continue
    end

    %Define polyshpae to do simple intersect check
    psCB(i) = polyshape(CB{i}(1,:),CB{i}(2,:));

    %Check if vertices are contained in any existing obstacles
    noIntersect = true;
    for k = 1:(i-1)
        [polyout,shapeID,vertexID] = intersect(psCB(k),psCB(i));
        if ~isempty(shapeID)
            %Intersect Occured
            noIntersect = false;
            break
        end
    end
    if ~noIntersect
        % Polygon intersects with at least one other polygon
        % Try again
        CB(i)= [];
        psCB(i) = [];
        continue
    end

    % Plot the Obstacle
    try
        ptc.cb(i) = plotCObstacle(CB{i},i);
    catch
        fprintf([...
            'Unable to plot C-obstacle %d.\n',...
            ' -> Make sure "plotCObstacle.m" is in the current path.\n'],i);
    end
    drawnow;

    % Establish break condition
    if i >= nObs
        break
    end

    % Iterate
    i = i+1;
end

%% Define q_init and q_goal within bounds

% Orientation of q is fixed with respect to the given C-obstacles

% Number of q's
nq = 2; % user-defined number of waypoints (nq = 2 for q_init & q_goal)

% Define obstacle limits
xx = [min(bounds(1,:)),max(bounds(1,:))];
yy = [min(bounds(2,:)),max(bounds(2,:))];

i = 1;
while true
    % Generate random configuration
    q{i} = [diff(xx); diff(yy)].*rand(2,1) + [xx(1); yy(1)];

    % Check if q lies within bounds
    isInBounds = true;
    if ~isinterior(psBnds,q{i}(1),q{i}(2))
        %Remove q
        isInBounds = false;
    end
    if ~isInBounds
        q(i) = [];
        continue
    end

    % Check if q lies within any C-Obstacles
    isInObstacle = false;
    for k = 1:numel(CB)
        if isinterior(psCB(k),q{i}(1),q{i}(2))
            %Remove q
            isInObstacle = true;
            break
        end
    end
    if isInObstacle
        q(i) = [];
        continue
    end

    % Define break condition
    if i >= nq
        break
    end

    i = i+1;
end

% Defining q_init and q_goal using successful q's from above
q_init = q{1};
q_goal = q{end};
q_wpnt = {};
if nq > 2
    q_wpnt = q(2:end-1);
end

% Plotting q init and q goal
plt.init = plot(axs,q_init(1),q_init(2),'or',...
    'MarkerSize',8,'LineWidth',2,'MarkerFaceColor','g');
plt.goal = plot(axs,q_goal(1),q_goal(2),'xr',...
    'MarkerSize',8,'LineWidth',2);

%% Update axes limits
xlim(axs,xx);
ylim(axs,yy);

%% Clean up workspace
clearvars -except q_init q_goal q_wpnt CB bounds fig axs ptc plt