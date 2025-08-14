% Isaac Shaw
% Robot Motion Planning
% 8/13/2025

function qpath = rrtDiffDrive(A, q_init, q_goal, B, bounds, options)

    arguments
        A (2, :) double
        q_init (3,1) double
        q_goal (3,1) double
        B cell

        bounds (2, :) double = [0; 0]

        options.debug logical = false
        options.maxIter (1,1) double = 5000
        options.stepSize (1,1) double = 0.5
        options.goalBias (1,1) double = 0.1

        options.omegaMax (1,1) double = pi/2
        options.vMax (1,1) double = 1
        options.wTheta (1,1) double = 0.5

        options.numCollisionSteps (1,1) double = 20
        options.maxTurnPerStep (1,1) double = .1      % also used to sample C-obs orientations
        options.deltaT (1,1) double = 0.1
        options.considerBounds logical = true

        % Goal-region controls (same spirit as rayCast)
        options.goalRegionNRays (1,1) double = 36
        options.goalAngleSteps (1,1) double = 36
        options.goalLineSteps  (1,1) double = 50

        options.steeringMode (1,:) char {mustBeMember(options.steeringMode, {'dubins','reeds-shepp'})} = 'reeds-shepp'
    end

    % --- Input checks ---
    checkQ(q_init); checkQ(q_goal);
    checkVertices(A); checkVertices(bounds);

    if ~inpolygon(q_init(1), q_init(2), bounds(1,:), bounds(2,:)) || ...
       ~inpolygon(q_goal(1), q_goal(2), bounds(1,:), bounds(2,:))
        error('All points must be inside the boundary')
    end
    for i = 1:numel(B)
        checkVertices(B{i});
        if ~all(inpolygon(B{i}(1,:), B{i}(2,:), bounds(1,:), bounds(2,:)))
            error('All obstacle vertices must be inside the boundary')
        end
    end

    % Robot size and bounds prep
    [cx, cy] = centroid(polyshape(A.', 'Simplify', true));
    R_robot = max(sqrt((A(1,:) - cx).^2 + (A(2,:) - cy).^2));

    if isequal(bounds, [0;0])

        verts = [q_init(1:2,:), q_goal(1:2,:)];

        for i = 1:numel(B)
            verts = [verts, B{i}]
        end

        xlim = [min(verts(1,:)) - 2.5*R_robot, max(verts(1,:) + 2.5*R_robot)];
        ylim = [min(verts(2,:)) - 2.5*R_robot, max(verts(2,:) + 2.5*R_robot)];

        bounds = [xlim; ylim];
    end

    boundsPS = polyshape(bounds.', 'Simplify', false);

    % Compute obstacle union for convenience when checking collisions
    obstacles = cell(1, numel(B));
    obstacles_union = polyshape;
    for i = 1:numel(B)
        obstacles{i} = polyshape(B{i}.', 'Simplify', false);
        obstacles_union = union(obstacles_union, obstacles{i});
    end

    % Precompute the C-Space obstacles for some discrete thetas
    thetas = linspace(0, 2*pi, max(3, ceil(1/options.maxTurnPerStep)));
    cObs = cell(1, numel(B));
    for i = 1:numel(B)
        allV = [];
        for t = 1:numel(thetas)
            allV = [allV, cObstacle(thetas(t), A, B{i})]; %#ok<AGROW>
        end
        allV = rmmissing(allV, 2);
        if ~isempty(allV)
            allV = uniquetol(allV.', 1e-4, 'ByRows', true).';
            if size(allV,2) >= 3
                k = convhull(allV(1,:).', allV(2,:).'); k(end) = [];
                cObs{i} = allV(:, k);
            else
                cObs{i} = allV;
            end
        else
            cObs{i} = [];
        end
    end

    % --- Goal region polygon (exactly like rayCast flow) ---
    Lmax = max(1, 3*(max(bounds(1,:)) - min(bounds(1,:))) + ...
                  3*(max(bounds(2,:)) - min(bounds(2,:))));
    % If goal inside any C-obstacle, fail fast
    for k = 1:numel(cObs)
        if ~isempty(cObs{k}) && inpolygon(q_goal(1), q_goal(2), cObs{k}(1,:), cObs{k}(2,:))
            qpath = []; return
        end
    end
    goalPoly = getGoalRegion(q_goal(1:2), cObs, bounds, options.goalRegionNRays, Lmax);

    % General sampling limits (samples filtered if considerBounds option)
    x_lims = [min(bounds(1,:)), max(bounds(1,:))];
    y_lims = [min(bounds(2,:)), max(bounds(2,:))];

    nodes(1) = struct('q', q_init, 'parent', 0, 'trajToParent', [], 'children', []);
    foundPath = false; goalId = NaN;

    % Debug plotting
    if options.debug
        if isempty(groot().Children)
            figure('Name','RRT - DEBUG');
        end

        plot(boundsPS, 'FaceColor','none','EdgeColor','k');

        [finit, ~] = plotRobot(q_init, A, 'world', true);
        [fgoal, ~] = plotRobot(q_goal, A, 'world', true);

        set(finit, 'Parent', hgtransform('Matrix', Tz(q_init(3))));
        set(fgoal, 'Parent', hgtransform('Matrix', Tz(q_goal(3))));

        for i = 1:numel(B)
            plotObstacle(B{i}, i);
        end

        for i = 1:numel(cObs)
            p = plotCObstacle(cObs{i}, i);
            set(p, 'FaceColor', 'y')
        end

        plot(q_init(1), q_init(2), 'dg', 'MarkerSize',10, 'LineWidth',2);
        plot(q_goal(1), q_goal(2), 'dr', 'MarkerSize',10, 'LineWidth',2);

        if ~isempty(goalPoly)
            plot([goalPoly(1,:) goalPoly(1,1)], [goalPoly(2,:) goalPoly(2,1)], 'm--', 'LineWidth',5);
        end

        drawnow limitrate;
    end


    % --- Main loop ---
    for iter = 1:options.maxIter
        % Sample (with goal bias)
        if rand < options.goalBias
            q_rand = q_goal;
        else
            while true
                x = x_lims(1) + rand * diff(x_lims);
                y = y_lims(1) + rand * diff(y_lims);
                if ~options.considerBounds || inpolygon(x,y,bounds(1,:),bounds(2,:)), break; end
            end
            q_rand = [x; y; 2*pi*rand];
        end

        % Nearest
        [~, nearId] = min(arrayfun(@(n) configDist(n.q, q_rand, options.wTheta), nodes));
        q_near = nodes(nearId).q;

        % --- Steer using differential-drive kinematics (unicycle model) ---
        % ẋ = v cosθ, ẏ = v sinθ, θ̇ = ω with |v|≤vMax, |ω|≤ωMax.
        % Reeds–Shepp allows v<0 (reverse), Dubins forces v≥0 with in-place pivots when needed.
        [q_new, traj, isFree] = steer(q_near, q_rand, options.stepSize, ...
                                      options.vMax, options.omegaMax, ...
                                      A, obstacles_union, options.numCollisionSteps, ...
                                      options.deltaT, bounds, options.considerBounds, ...
                                      options.steeringMode);
        if ~isFree || isempty(q_new), continue; end

        % Add node
        newId = numel(nodes) + 1;
        nodes(newId) = struct('q', q_new, 'parent', nearId, 'trajToParent', traj, 'children', []);
        nodes(nearId).children(end+1) = newId;
        if options.debug, plot(traj(1,:), traj(2,:), 'b-', 'LineWidth', 1); drawnow; end

        % --- Goal region check (polygon, like rayCast) ---
        if ~isempty(goalPoly) && inpolygon(q_new(1), q_new(2), goalPoly(1,:), goalPoly(2,:))
            [traj_final, ok] = finalApproach(q_new, q_goal, A, obstacles_union, bounds, ...
                                             options.considerBounds, options.goalAngleSteps, options.goalLineSteps);
            if ok
                goalId = numel(nodes) + 1;
                nodes(goalId) = struct('q', q_goal, 'parent', newId, 'trajToParent', traj_final, 'children', []);
                if options.debug, plot(traj_final(1,:), traj_final(2,:), 'r-', 'LineWidth', 2); drawnow; end
                foundPath = true; break;
            end
        end

        % --- Fallback: if very close in config metric, try a direct steer to goal ---
        if configDist(q_new, q_goal, options.wTheta) < options.stepSize
            [q_new_goal, traj_goal, okGoal] = steer(q_new, q_goal, options.stepSize, ...
                                                    options.vMax, options.omegaMax, ...
                                                    A, obstacles_union, options.numCollisionSteps, ...
                                                    options.deltaT, bounds, options.considerBounds, ...
                                                    options.steeringMode);
            if okGoal && ~isempty(q_new_goal)
                goalId = numel(nodes) + 1;
                nodes(goalId) = struct('q', q_goal, 'parent', newId, 'trajToParent', traj_goal, 'children', []);
                if options.debug, plot(traj_goal(1,:), traj_goal(2,:), 'r-', 'LineWidth', 2); drawnow; end
                foundPath = true; break;
            end
        end
    end

    % --- Extract path ---
    if ~foundPath, qpath = []; return; end
    idxs = goalId; cur = nodes(goalId).parent;
    while cur ~= 0, idxs(end+1) = cur; cur = nodes(cur).parent; end %#ok<AGROW>
    idxs = fliplr(idxs);
    qpath = nodes(idxs(1)).q;
    for k = 2:numel(idxs)
        seg = nodes(idxs(k)).trajToParent;
        if isempty(seg), seg = nodes(idxs(k)).q; end
        qpath = [qpath, seg(:,2:end)]; %#ok<AGROW>
    end
    qpath(:,1)   = q_init;
    qpath(:,end) = q_goal;
end
