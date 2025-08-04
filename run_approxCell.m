% This file runs all 3 methods explored for the final project and visualizes the results. 
close all
clear all
addpath("TransformationToolboxFunctions\");
addpath("utilities\");


seeds = [2, 3, 4, 7, 9, 10, 20, 93400];

% seeds = 1:10000
% for i=1:numel(seeds)
%     close all
%     seed = seeds(i)
%     [A, B, q_init, q_goal, bounds] = createEnvironment('seed', seed, 'plotEnvironment', true);
%     pause

% end 

seed = seeds(end-2);

[A, B, q_init, q_goal, bounds] = createEnvironment('seed', seed, 'numObstacles', 5);

tic
[adj, wAdj, xyq] = approxCellGraph3d(A, q_init, q_goal, B, bounds, 'considerBounds', true, 'debug', false, 'minRectScale', 30, 'debugPlotEmptiesAdjacencyCheck', false);
toc
% plotAdjacency3d(adj, xyq)

heuristic = [];

for i=1:size(xyq, 2)
    next_i = i+1;
    if i == size(xyq,2)
        next_i = 1;
    end
    heuristic(end+1) = norm(xyq(:, i) - xyq(:,next_i));
end

path = Astar(adj, heuristic)
pathpoints = xyq(:,path)
q_init
q_goal

%% Plot the path
[Fa, ~] = plotRobot(q_init, A, "world", true);

d = sqrt(sum(diff(pathpoints,1,2).^2,1))
t = [0, cumsum(d)];
tq = linspace(t(1), t(end), 400);

% %% Linear Interpolation Path
% all_qs_linear_inperp = [interp1(t, pathpoints(1,:), tq, 'linear');...
%                         interp1(t, pathpoints(2,:), tq, 'linear');...
%                         interp1(t, pathpoints(3,:), tq, 'linear')];

% % Plot linear interpolated path
% plotPath(all_qs_linear_inperp, Fa, A, B)


%% Pivot Drive Pivot

maxDistPerStep  = 0.5;
maxTurnPerStep  = 0.1;

all_qs_pdp_path = pathpoints(:,1);  % start exactly at first waypoint

for idx = 1:size(pathpoints,2)-1

    currentWaypoint = pathpoints(:,idx);
    nextWaypoint = pathpoints(:, idx+1);

    numTurnSteps = max(ceil(abs(nextWaypoint(3) - currentWaypoint(3))/maxTurnPerStep), 5);
    pivot = zeros(3, numTurnSteps);
    pivot(3,:) = linspace(currentWaypoint(3), nextWaypoint(3), numTurnSteps);
    pivot(1,:) = currentWaypoint(1);
    pivot(2,:) = currentWaypoint(2);

    
    numTravelSteps = max(ceil(d(idx)/maxDistPerStep), 5);
    drive = [linspace(currentWaypoint(1), nextWaypoint(1), numTravelSteps); linspace(currentWaypoint(2), nextWaypoint(2), numTravelSteps)];
    drive(3,:) = nextWaypoint(3);
    all_qs_pdp_path = [all_qs_pdp_path, pivot, drive];

    currentWaypoint, nextWaypoint
    pivot
    drive

end

currentWaypoint = pathpoints(:,end-1);
nextWaypoint = pathpoints(:, end);

numTurnSteps = max(ceil(abs(nextWaypoint(3) - currentWaypoint(3))/maxTurnPerStep), 5);
pivot = zeros(3, numTurnSteps);
pivot(3,:) = linspace(currentWaypoint(3), nextWaypoint(3), numTurnSteps);
pivot(1,:) = nextWaypoint(1);
pivot(2,:) = nextWaypoint(2);

all_qs_pdp_path = [all_qs_pdp_path, pivot];

while true
    plotPath(all_qs_pdp_path, Fa, A, B, 'wait', .05);
end


%% Custom Spline Path


all_qs_spline = [ppval(spline(t, pathpoints(1,:)), tq);...
                 ppval(spline(t, pathpoints(2,:)), tq);...
                 ppval(spline(t, pathpoints(3,:)), tq) ];

% Plot "custom" spline path
plotPath(all_qs_spline, Fa, A, B)

%% Curve Fitting Toolbox Path

C   = cscvn(pathpoints);
tq  = linspace(C.breaks(1), C.breaks(end), 400);
all_qs_cscvn = fnval(C, tq);

plotPath(all_qs_cscvn, Fa, A, B)


function plotPath(waypoints, robotFrame, robot, obstacles, options)
    arguments
        waypoints (3,:) double
        robotFrame handle
        robot (2,:) double
        obstacles cell
        options.wait (1,1) double = 0
    end

    Cobstacles = {};

    for i=1:size(waypoints,2)
        theta = waypoints(3,i);
        CBs = cellfun(@(b) cObstacle(theta, robot, b), obstacles, 'UniformOutput', false);

        Cobstacles{i} = CBs;
    end

    obs_patches = [];
    for j = 1:numel(obstacles)
        [obs, l] = plotCObstacle(Cobstacles{1}{j}, j);
        obs_patches(:, end+1) = [obs; l];
    end

    for i=1:size(waypoints, 2)

        for j = 1:numel(obstacles)
            set(obs_patches(1, j), 'Vertices', Cobstacles{i}{j}.');
        end

        q = waypoints(:, i);
        h = qToH(q);
        set(robotFrame, 'Matrix', h);
        drawnow;
        if options.wait > 0
            pause(options.wait)
        end
    end

    for j = 1:numel(obstacles)
        delete(obs_patches(:,j));
    end

end 

% Get transform from config
function H = qToH(q)
    X = q(1:2, :);
    k = q(3);
    R = [cos(k) -sin(k); sin(k) cos(k)];
    H = eye(4,4);
    H(1:2, 1:2) = R;
    H(1:2,4) = X;
end
