
function [A, B, q_init, q_goal, bounds] = createEnvironment(options)
    %% createEnvironment
    % This function generates a random convex boundary, random nonoverlapping obstacles, and random starting and ending positions and orientations for a robot
    % This is an adaptation of Dr. Kutzers createRandomEnviornment.m

    arguments
        options.seed (1,1) double = 2;

        options.robotSides (1,1) double {mustBeGreaterThan(options.robotSides, 2)} = 5; 
        options.regularRobot (1,1) = false;
        options.robotOrientation (1,1) = 'r';

        options.maxObsSides (1,1) double {mustBeGreaterThan(options.maxObsSides, 2)} = 8;
        options.minObsSides (1,1) double {mustBeGreaterThan(options.minObsSides, 2)} = 5;
        options.numObstacles (1,1) double {mustBePositive, mustBeInteger} = 10;
        options.boundaryBuffer (1,1) double = 1;
        options.regularObstacles (1,1) = false;

        options.maxBoundSides (1,1) double {mustBeGreaterThan(options.maxBoundSides, 2)} = 10;
        options.maxEnvironmentRadius (1,1) double = 20; 

        options.plotEnvironment (1,1) = true;

        options.maxPolySize (1,1) double = 1.5;
        options.minPolySize (1,1) double = .75;

    end

    rng(options.seed);

    allPoints = [];
    B = {};
    allPolyshapes = polyshape;
    for i =1:options.numObstacles
        while true
            numSides = randi([options.minObsSides, options.maxObsSides], 1);
            obsRadius = rand * (options.maxPolySize - options.minPolySize) + options.minPolySize;

            radius = rand * (options.maxEnvironmentRadius - options.maxPolySize);

            angle = rand * 2*pi;
            centroid = radius * [cos(angle); sin(angle)];
            bi = getConvexPolyPoints(numSides, 'centroid', centroid, 'radius', obsRadius, 'regular', options.regularObstacles);
            bi_ps = polyshape(bi.');
            intersection_obs = intersect(bi_ps, allPolyshapes);

            if intersection_obs.NumRegions == 0
                allPolyshapes = union(allPolyshapes, bi_ps);
                B{i} = bi;
                allPoints = [allPoints, bi];
                break
            end
        end
    end

    %% Define q_init and q_goal within bounds

    % Define a robot 
    robot_radius = rand * (options.maxPolySize - options.minPolySize) + options.minPolySize;

    A = getConvexPolyPoints(options.robotSides, 'regular', options.regularRobot, 'radius', robot_radius);

    q_init = [];
    q_goal = [];

    while true
        angles = rand(1,2) * 2*pi;

        xy = [cos(angles); sin(angles)] .* (rand(1,2) * (options.maxEnvironmentRadius - options.maxPolySize));

        if options.robotOrientation == 'r'
            orientation = 2*pi*rand(1,2);
        elseif isnumeric(options.robotOrientation)
            orientation = [options.robotOrientation, options.robotOrientation];
        end

        q_init = [xy(:,1); orientation(1)];
        A_init = moveRobot(q_init, A, 'world', true);
        q_goal = [xy(:,2); orientation(2)];
        A_goal = moveRobot(q_goal, A, 'world', true);

        A_init_ps = polyshape(A_init.');
        A_goal_ps = polyshape(A_goal.');

        int_init = intersect(A_init_ps, allPolyshapes);
        int_goal = intersect(A_goal_ps, allPolyshapes);
        int_init_goal = intersect(A_goal_ps, A_init_ps);

        if int_init.NumRegions == 0 && int_goal.NumRegions == 0 && int_init_goal.NumRegions == 0
            allPoints = [allPoints, A_init, A_goal];
            break
        end
    end

    % Get bounds of environment
    k = convhull(allPoints.');
    k(end) = [];

    bounds = allPoints(:, k);
    bnds_ps_vertices = polybuffer(polyshape(bounds.'), options.boundaryBuffer, "JointType", "square").Vertices.';
    k = convhull(bnds_ps_vertices.');
    k(end) = [];
    bounds = bnds_ps_vertices(:,k);
    % [cx, cy] = bnds_ps.centroid;
    % % inflate the workspace by 10% so boundary doesn't sit on obs or robot
    % while true

    %     bounds_temp = ((bounds.' - [cx, cy]) * 1.1 + [cx cy]).';

    %     k = convhull(bounds_temp.');
    %     k(end) = [];
    %     bounds_temp = bounds_temp(:,k);
    %     psBnds = polyshape(bounds_temp.');

    %     tfIn = isinterior(psBnds, allPoints(1,:), allPoints(2,:));
    %     if all(tfIn)
    %         bounds = bounds_temp;
    %         break
    %     end
    % end

    % Enforce bound side requirements
    if size(bounds,1) > options.maxBoundSides
        while size(bounds,1) > options.maxBoundSides
            m = size(bounds,1);
            ang = zeros(m,1);
            for i = 1:m
                a = bounds(mod(i-2,m)+1,:) - bounds(i,:);
                b = bounds(mod(i,  m)+1,:) - bounds(i,:);
                ang(i) = abs(atan2(det([a;b]), dot(a,b)));
            end
            [~,j] = min(ang);
            bounds(j,:) = [];
        end
    end

    %% Plot enviornoment if desired
    if options.plotEnvironment
        if isempty(groot().Children)
            fig = figure('Name', 'Random Environment');
        else
            fig = gcf;
        end
        axs = gca;
        % Plot bounds
        psBnds = polyshape(bounds.');
        pltBnds = plot(axs,psBnds,'FaceColor','k','EdgeColor','k','FaceAlpha',0.1, 'Tag', 'Bounds');
        % bndsplt = plot(axs, bounds(1,:), bounds(2,:),'FaceColor','k','EdgeColor','k','FaceAlpha',0.1, 'Tag', 'Bounds');
        
        % Plot obstacles
        for i=1:numel(B)
            pto(i) = plotObstacle(B{i}, i);
        end
        
        % Plot robot
        plt_init = plotRobot(q_init, A, 'text', 'q_{init}', 'world', true);
        plt_goal = plotRobot(q_goal, A, "text", 'q_{goal}', 'alpha', .2, 'world', true);
        
        %% Update axes limits
        xlim(axs,[min(bounds(1,:)), max(bounds(1,:))]);
        ylim(axs,[min(bounds(2,:)), max(bounds(2,:))]);
    end
end