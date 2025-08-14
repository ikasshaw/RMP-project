close all
clear all

addpath("TransformationToolboxFunctions\");
addpath("utilities\shared");
addpath("utilities\rayCast");

seeds = [2, 3, 4, 7, 9, 10, 20, 93400];

for i = 4:10
    for seedid = 1:numel(seeds)

        close all
        clearvars -except seeds seedid i

        [A, B, q_init, q_goal, bounds] = createEnvironment('seed', seed, 'numObstacles', i);

        tic

        q_path = rayCastPlanner(A, q_init, q_goal, B, bounds, 'debug', false);

        % q_path = rayCastPlanner(A, q_init, q_goal, B, bounds, 'debug', true, 'debugPause', .001, 'debugStep', true);

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


