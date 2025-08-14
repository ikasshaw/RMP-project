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