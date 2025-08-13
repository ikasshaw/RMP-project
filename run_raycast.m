close all
clear all

addpath("TransformationToolboxFunctions\");
addpath("utilities\shared");
addpath("utilities\rayCast");


seeds = [2, 3, 4, 7, 9, 10, 20, 93400];

% seed = seeds(end-2);
for i = 1:10
    for seedid = 1:numel(seeds)

        seed = seeds(seedid);
        [A, B, q_init, q_goal, bounds] = createEnvironment('seed', seed, 'numObstacles', i);%, 'regularRobot', false, 'robotOrientation', 0);
        % [A, B, q_init, q_goal, bounds] = createEnvironment('seed', seed, 'numObstacles', 10);%, 'regularRobot', false, 'robotOrientation', 0);

        tic
        % q_path = rayCastPlanner(A, q_goal, q_init, B, bounds, 'debug', false)
        % q_path = rayCastPlanner(A, q_init, q_goal, B, bounds, 'debug', false);
        % q_path = rayCastPlanner(A, q_init, q_goal, B, bounds, 'debug', true);
        q_path = rayCastPlanner(A, q_init, q_goal, B, bounds, 'debug', true, 'debugPause', 0, 'debugStep', true);
        % q_path = rayCastPlanner(A, q_init, q_goal, B, bounds, 'debug', true, 'debugPause', .01, 'debugStep', true);
        % q_path = rayCastPlanner(A, q_init, q_goal, B, bounds,'debug', true, 'debugPause', .01, 'debugStep', true, 'debugCObs', false);
        toc

        %% Plot the path
        [Fa, ~] = plotRobot(q_init, A, "world", true);

        if size(q_path, 2) >= 2
            fprintf('Found path for seed %d with %d obstacles\n', i, seed)
            plotPath(q_path, Fa, A, B)
        else
            fprintf('No path for seed %d with %d obstacles\n', i, seed)
        end

    end

end

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
