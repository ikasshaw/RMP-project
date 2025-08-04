% Isaac Shaw
% Robot Motion Planning
% 8/10/2025

function qpath = rayCastPlanner(A, q_init, q_goal, B, bounds, options)

    arguments
        A (2,:) double
        q_init (3,1) double
        q_goal (3,1) double
        B cell
        bounds (2,:) double = [0; 0]
        
        options.rngSeed (1,1) double = 1

        options.initialRays (1,1) double = 5
        options.maxBounces (1,1) double {mustBeInteger,mustBeNonnegative} = 30
        options.maxIter (1,1) double = 5000
        
        options.goalRegionNRays (1,1) double = 360
        options.goalRegionMargin (1,1) double = 1e-3
        options.goalThetaTol (1,1) double = deg2rad(10)

        options.includeGoalReflection logical = true
        options.includeSpecularReflection logical = true
        options.includeDiffuseReflection logical = true
        options.includeRandomReflection logical = true
        options.numberOfDiffuse (1,1) double {mustBeInteger,mustBePositive} = 5

        options.turnCheckSteps (1,1) double {mustBeInteger,mustBePositive} = 12
        options.maxTurnPerStep (1,1) double = .1
        options.maxDistPerStep (1,1) double = .1
        options.scoreChildren logical = false        
        
        options.goalAngleSteps (1,1) double = 36
        options.goalLineSteps (1,1) double = 50
        
        options.debug logical = false
        options.debugCObs logical = true
        options.debugStep logical = true
        options.debugPause (1,1) double = 0
        options.OnlyCurrentHitRay logical = false

        options.margin (1,1) double = 1e-3
    end

    % --- Input checks ---
    checkQ(q_init);
    checkQ(q_goal);
    checkVertices(A);
    checkVertices(bounds);

    if ~inpolygon(q_init(1), q_init(2), bounds(1,:), bounds(2,:)) || ...
       ~inpolygon(q_goal(1), q_goal(2), bounds(1,:), bounds(2,:))
        error('All points must be inside the boundary');
    end

    for i = 1:numel(B)
        checkVertices(B{i});
        if ~all(inpolygon(B{i}(1,:), B{i}(2,:), bounds(1,:), bounds(2,:)))
            error('All obstacle vertices must be inside the boundary');
        end
    end

    childrenPerBounce = sum([options.includeGoalReflection, options.includeSpecularReflection, options.includeDiffuseReflection, options.includeRandomReflection, options.numberOfDiffuse]);

    % Compute a conservative robot radius
    R_robot = max(sqrt(sum(A.^2,1)));

    % If bounds not provided, compute from environment with generous margin for robot size
    if isequal(bounds, [0;0])
        verts = [q_init(1:2,:), q_goal(1:2,:)];
        for i = 1:numel(B)
            verts = [verts, B{i}];
        end

        vertices_x_lims = [min(verts(1,:)) - 2.5*R_robot, max(verts(1,:) + 2.5*R_robot)];
        vertices_y_lims = [min(verts(2,:)) - 2.5*R_robot, max(verts(2,:) + 2.5*R_robot)];
        bounds = [vertices_x_lims; vertices_y_lims];
    end

    % Max line length for ray casting
    Lmax = max(1, 3*(max(bounds(1,:)) - min(bounds(1,:))) + ...
                   3*(max(bounds(2,:)) - min(bounds(2,:))));  % generous length beyond world extent

    % Precompute boundary edges for convex checks
    boundsE = edgesFromVertices(bounds);

    rng(options.rngSeed);

    R_robot = max(sqrt(sum(A.^2,1)));

    % Debug bounds shape only
    boundsPS = polyshape(bounds.', 'Simplify', false);

    % Compute a safe goal region to aim for
    % If this region is intersected by a ray, we know that we can safely reach the goal with a pivot drive pivot maneuver
    d_ray = radialClearance(q_goal(1:2), B, bounds, options.goalRegionNRays, Lmax);
    r_cap = max(0, d_ray - (R_robot + options.goalRegionMargin));

    if r_cap > 0
        Kgoal = 32;
        thg = linspace(0, 2*pi, Kgoal+1);
        thg(end) = [];
        goalPoly = [q_goal(1) + r_cap*cos(thg);
                    q_goal(2) + r_cap*sin(thg)];
    else
        goalPoly = [];
    end

    % Debug plotting
    if options.debug
        if isempty(groot().Children)
            figure('Name','rayCastPlanner - DEBUG');
        end

        plot(boundsPS, 'FaceColor','none','EdgeColor','k');

        [finit, ~] = plotRobot(q_init, A, 'world', true);
        [fgoal, ~] = plotRobot(q_goal, A, 'world', true);

        set(finit, 'Parent', hgtransform('Matrix', Tz(q_init(3))));
        set(fgoal, 'Parent', hgtransform('Matrix', Tz(q_goal(3))));

        for i = 1:numel(B)
            plotObstacle(B{i}, i);
        end

        plot(q_init(1), q_init(2), 'dg', 'MarkerSize',10, 'LineWidth',2);
        plot(q_goal(1), q_goal(2), 'dr', 'MarkerSize',10, 'LineWidth',2);

        if ~isempty(goalPoly)
            plot([goalPoly(1,:) goalPoly(1,1)], [goalPoly(2,:) goalPoly(2,1)], 'm--', 'LineWidth',1.5);
        end

        drawnow limitrate;
    end

    StackItem = @(p,th,b,segs,rtype,htype,g) struct('p',p,'th',th,'bounces',b,'segs',segs,'rayType',rtype,'hitType',htype,'len',g);

    th0 = wrapToPi(q_init(3));
    th_goal = wrapToPi(atan2(q_goal(2) - q_init(2), q_goal(1) - q_init(1)));

    % Initial set of rays to cast and includes direct to goal, initial direction, and some random
    ths = wrapToPi(th0 + (2*pi)*rand(1,options.initialRays));
    dirs = [ths, th0, th_goal];

    maxStack = max(1000, options.maxIter + options.maxBounces*childrenPerBounce*10);
    stack = repmat(StackItem([0;0],0,0,zeros(3,1),RayType.initial,HitType.none,0), 1, maxStack);
    stackTop = 1;

    % Stack items initialized with initial position, direction, bounce count, segments, ray type, hit type, cost, length
    stack(stackTop) = StackItem(q_init(1:2), th0, 0, q_init, RayType.initial, HitType.none, 0);

    for i = 2:numel(dirs)
        stackTop = stackTop + 1;
        stack(stackTop) = StackItem(q_init(1:2), dirs(i), 0, q_init, RayType.initial, HitType.none, 0);
    end

    successPaths = cell(1, options.maxIter);
    nSuccess = 0;

    % Best-so-far + visited states
    bestLen = inf;
    stateXYRes = 0.02;
    stateThRes = deg2rad(3);
    visited = containers.Map('KeyType','char','ValueType','logical');

    makeKey = @(p,th,b) sprintf('%d_%d_%d_%d', ...
                    round(p(1)/stateXYRes), round(p(2)/stateXYRes), ...
                    round(wrapToPi(th)/stateThRes), b);

    for i = 1:stackTop
        visited(makeKey(stack(i).p, stack(i).th, stack(i).bounces)) = true;
    end

    iters = 0;
    cObs = cell(1,numel(B));

    while (stackTop > 0) && (iters < options.maxIter)

        node  = stack(stackTop);
        stackTop = stackTop - 1;
        iters = iters + 1;

        p = node.p;
        th = wrapToPi(node.th);
        bounces = node.bounces;
        segs = node.segs;
        len = node.len;

        % Branch-and-bound lower bound
        h = max(0, norm(p - q_goal(1:2)) - r_cap);
        if (len + h) >= bestLen
            continue
        end

        if options.debug
            delete(findall(gca, 'Tag','ParentPath'));
            for i = 1:(size(segs, 2)-1)
                plot(segs(1, i:i+1), segs(2,i:i+1), 'm-', 'LineWidth', 1.5, 'Tag', 'ParentPath');
            end
        end

        while (bounces <= options.maxBounces)

            % C-obstacles for this theta
            for i = 1:numel(B)
                cObs{i} = cObstacle(th, A, B{i});
            end

            if options.debug && options.debugCObs
                delete(findall(gca, 'Tag', "CObsLabel"));
                delete(findall(gca, 'Tag', "CObs"));
                debugPlotCspaceAtTheta(cObs);
                drawnow limitrate;
            end

            [hitType, hitPoint, segRay, hitNormal] = castRay(p, th, cObs, bounds, goalPoly, Lmax);

            ray_len = [];
            if ~isempty(segRay)
                ray_len = norm(segRay(:,2) - segRay(:,1));
            end

            if hitType == HitType.goal

                segs = [segs, [hitPoint(1); hitPoint(2); th], q_goal];

                len + ray_len + norm(segs(1:2,end) - hitPoint);
                bestLen = min(bestLen, len);
                nSuccess = nSuccess + 1;
                successPaths{nSuccess} = segs;

                fprintf('Solution found: path length = %.4f (success #%d)\n', len, nSuccess);

                if options.debug
                    plot(hitPoint(1), hitPoint(2), 'ro', 'MarkerSize',6, 'LineWidth',1.5);
                    for j = 1:(size(segs, 2)-1)
                        plot(segs(1, j:j+1), segs(2,j:j+1), 'g-', 'LineWidth', 1.5, 'Tag', 'GoalPath');
                    end
                end
                break
            end

            childRays = genChildren(hitType, ...
                                    th, ...
                                    hitNormal, ...
                                    hitPoint, ...
                                    q_goal, ...
                                    options.includeGoalReflection, ...
                                    options.includeSpecularReflection, ...
                                    options.includeDiffuseReflection, ...
                                    options.numberOfDiffuse, ...
                                    options.includeRandomReflection, ...
                                    R_robot, ...
                                    A, ...
                                    B, ...
                                    bounds, ...
                                    options.maxTurnPerStep, ...
                                    ray_len, ...
                                    childrenPerBounce);

            if isempty(childRays)
                break
            end

            % Filter near-parallel
            keep = true(1,numel(childRays));
            for k = 1:numel(childRays)
                if abs(wrapToPi(childRays(k).th - th)) < 1e-3
                    keep(k) = false;
                end
            end
            childRays = childRays(keep);

            if isempty(childRays)
                if options.debug
                    fprintf('No children generated for bounce %d at point (%.2f, %.2f) with theta %.2f\n', bounces, p(1), p(2), th);
                end
                break
            end

            % Score children with a made up heuristic...
            if options.scoreChildren
                w_turn = 1.0;
                w_goal = 0.7;
                w_type = 0.3;

                u_parent = [cos(th); sin(th)];
                v_goal = q_goal(1:2) - hitPoint;
                norm_goal = norm(v_goal);
                if norm_goal > 0
                    u_goal = v_goal / norm_goal;
                else
                    u_goal = u_parent;
                end

                typeRank = @(rt) (rt==RayType.goal)*0 + (rt==RayType.specular)*1 + (rt==RayType.diffuse)*2 + (rt==RayType.random)*3;

                scores = zeros(1,numel(childRays));
                for k = 1:numel(childRays)
                    u_child = [cos(childRays(k).th); sin(childRays(k).th)];
                    turn_proxy  = 1 - max(-1, min(1, dot(u_child, u_parent)));
                    align_proxy = 1 - max(-1, min(1, dot(u_child, u_goal)));
                    scores(k) = w_turn*turn_proxy + w_goal*align_proxy + w_type*typeRank(childRays(k).rayType);
                end

                [~, ord] = sort(scores, 'ascend');
                childRays = childRays(ord);
                childRays = childRays(1:min(childrenPerBounce, numel(childRays)));
            end

            % Add children to stack only if they are not worse than the best path found so far and haven't been visited
            for k = numel(childRays):-1:2
                segs_i = [segs, [childRays(k).p_emit; childRays(k).th]];
                g_i = len + norm(childRays(k).p_emit - p);
                h_i = max(0, norm(childRays(k).p_emit - q_goal(1:2)) - r_cap);

                if (g_i + h_i) >= bestLen
                    continue
                end

                key = makeKey(childRays(k).p_emit, childRays(k).th, bounces+1);
                if isKey(visited, key)
                    continue
                end
                visited(key) = true;

                stackTop = stackTop + 1;
                if stackTop > numel(stack)
                    stack = [stack, repmat(StackItem([0;0],0,0,zeros(3,1),RayType.initial,HitType.none,0), 1, maxStack)];
                end
                stack(stackTop) = StackItem(childRays(k).p_emit, childRays(k).th, bounces+1, segs_i, childRays(k).rayType, childRays(k).hitType, g_i);
            end

            if options.debug
                tags = ["HitPoint","HitNormal","HitRay","BacktrackRay","ChildRays"];
                for tag = tags
                    delete(findall(gca, 'Tag', tag));
                end

                if ~isempty(segRay)
                    switch hitType
                        case HitType.goal, color = 'g'; lw = 2;
                        case HitType.obstacle, color = 'b'; lw = 1.5;
                        otherwise; color = 'r'; lw = 1;
                    end
                    if options.OnlyCurrentHitRay
                        delete(findall(gca,'Tag','HitRay'));
                    end
                    plot(segRay(1,:), segRay(2,:), color, 'LineWidth', lw, 'Tag','HitRay');
                end

                plot(hitPoint(1), hitPoint(2), 'ro', 'MarkerSize',6, 'LineWidth',1.5, 'Tag', 'HitPoint');
                if ~isempty(hitNormal)
                    L=0.6;
                    quiver(hitPoint(1), hitPoint(2), hitNormal(1)*L, hitNormal(2)*L, 0, 'Color', 'm', 'LineWidth', 1.5, 'Tag', 'HitNormal');
                end

                for k = numel(childRays):-1:1
                    switch childRays(k).rayType
                        case RayType.goal, color = 'g';
                        case RayType.specular, color = 'b';
                        case RayType.diffuse, color = 'm';
                        case RayType.random, color = 'y';
                        otherwise; color = 'k';
                    end
                    L = 0.75;
                    quiver(childRays(k).p_emit(1), childRays(k).p_emit(2), cos(childRays(k).th)*L, sin(childRays(k).th)*L, 0, 'Color', color, 'LineWidth', 2, 'Tag','ChildRays');
                end

                drawnow limitrate;
                if options.debugStep
                    debugStepBlock(options.debugStep, options.debugPause);
                end
            end

            segs = [segs, [childRays(1).p_emit; childRays(1).th]];
            len    = len + norm(childRays(1).p_emit - p);
            p    = childRays(1).p_emit;
            th   = childRays(1).th;
            bounces    = bounces + 1;

            visited(makeKey(p, th, bounces)) = true;

            h = max(0, norm(p - q_goal(1:2)) - r_cap);
            if (len + h) >= bestLen
                break
            end
        end
    end

    if nSuccess == 0
        qpath = [];
        return
    end

    successPaths = successPaths(1:nSuccess);

    % pick shortest
    minLen = inf;
    minIdx = 0;

    for i = 1:nSuccess
        len = 0;
        for j = 1:size(successPaths{i}, 2) - 1
            len = len + norm(successPaths{i}(1:2, j) - successPaths{i}(1:2, j+1));
        end
        if len < minLen
            minLen = len;
            minIdx = i;
        end
    end

    waypoints = successPaths{minIdx};
    qpath = waypoints(:, 1);

    d = sqrt(sum(diff(waypoints,1,2).^2,1));

    for idx = 1:size(waypoints,2)-1

        currentWaypoint = waypoints(:,idx);
        nextWaypoint    = waypoints(:,idx+1);

        numTurnSteps = max(ceil(abs(wrapToPi(nextWaypoint(3) - currentWaypoint(3)))/options.maxTurnPerStep), 5);
        angleSteps   = linspace(currentWaypoint(3), nextWaypoint(3), numTurnSteps);
        pivot        = repmat(currentWaypoint, 1, numel(angleSteps));
        pivot(3,:)   = angleSteps;

        qpath        = [qpath, pivot];

        if idx ~= size(waypoints, 2)
            numTravelSteps = max(ceil(d(idx)/options.maxDistPerStep), 5);
            drive = [linspace(currentWaypoint(1), nextWaypoint(1), numTravelSteps);
                     linspace(currentWaypoint(2), nextWaypoint(2), numTravelSteps)];
            drive(3,:) = nextWaypoint(3);
            qpath = [qpath, drive];
        end
    end
end

function childData = genChildren(hit_type, ...
                                 th_in, ...
                                 n_hat, ...
                                 parent_hit_point, ...
                                 q_goal, ...
                                 includeGoalReflection, ...
                                 includeSpecularReflection, ...
                                 includeDiffuseReflection, ...
                                 numberOfDiffuse, ...
                                 includeRandomReflection, ...
                                 R_robot, ...
                                 A, ...
                                 B, ...
                                 bounds, ...
                                 maxTurnPerStep, ...
                                 ray_len, ...
                                 childrenPerBounce)

    u_ray = [cos(th_in); sin(th_in)];
    childData = struct('th',{},'p_emit',{},'backSeg',{},'rayType',{},'hitType',{});

    function try_add(angleType, diffuse_offset)
        [p_emit, backSeg, th] = computeSafeEmitBacktrack( ...
            parent_hit_point, u_ray, th_in, A, B, bounds, ...
            R_robot, q_goal, angleType, n_hat, ...
            maxTurnPerStep, diffuse_offset, ray_len);

        if ~isempty(p_emit) && ~isempty(backSeg) && abs(wrapToPi(th - th_in)) > 1e-3
            childData(end+1).th = th;
            childData(end).p_emit = p_emit;
            childData(end).backSeg = backSeg;
            childData(end).rayType = angleType;
            childData(end).hitType = hit_type;
        end
    end

    if includeGoalReflection
        try_add(RayType.goal, 0);
    end

    if ~isempty(n_hat)
        if includeSpecularReflection
            try_add(RayType.specular, 0);
        end

        if includeRandomReflection
            try_add(RayType.random, 0);
        end

        if includeDiffuseReflection
            diffuse_angles = linspace(-pi/2, pi/2, numberOfDiffuse);
            for kk = 1:numel(diffuse_angles)
                k = diffuse_angles(kk) + (rand*0.1 - 0.05);
                try_add(RayType.diffuse, k);
            end
        end
    end

    % Local dedup (emit location+heading)
    if ~isempty(childData)
        epsXY = 1e-6;
        epsTh = 1e-3;
        keep = true(1,numel(childData));
        for i = 1:numel(childData)
            for j = 1:i-1
                if keep(j) && norm(childData(i).p_emit - childData(j).p_emit) < epsXY && ...
                             abs(wrapToPi(childData(i).th - childData(j).th)) < epsTh
                    keep(i) = false;
                    break
                end
            end
        end
        childData = childData(keep);
    end

    % Cap count here as well (safety)
    if ~isempty(childData)
        childData = childData(1:min(childrenPerBounce, numel(childData)));
    end
end

% Function backtracks along the ray to find a safe emit point
% Uses C-obstacles for obstacle checks and convex boundary for bounds.
function [p_emit, backSeg, th_try] = computeSafeEmitBacktrack(hitPoint, ...
                                                              u_ray, ...
                                                              th_cur, ...
                                                              A, ...
                                                              B, ...
                                                              bounds, ...
                                                              R_robot, ...
                                                              q_goal, ...
                                                              angleType, ...
                                                              n_hat, ...
                                                              maxTurnPerStep, ...
                                                              diffuse_offset, ...
                                                              ray_len)

    p_emit = [];
    backSeg = [];
    th_try = [];

    u = u_ray / max(norm(u_ray), eps);
    moveDir = -u;

    if isempty(ray_len) || ~isfinite(ray_len) || (ray_len <= 0)
        ray_len = 1.0;
    end
    
    % get percent of ray length robot size is
    rob_percent = R_robot / ray_len;
    % Backtrack step sizes as percents of ray length
    fracs = [rob_percent, 0.10, 0.20, 0.30, 0.40, 0.50];
    fracs = sort(fracs,"ascend");

    steps = max(1e-3, fracs * ray_len);

    boundsE = edgesFromVertices(bounds);

    if ~isempty(n_hat)
        normal_world_angle = atan2(n_hat(2), n_hat(1));
    else
        normal_world_angle = NaN;
    end

    for ii = 1:numel(steps)

        d = steps(ii);
        p_emit = hitPoint + d * moveDir;

        switch angleType
            case RayType.goal
                th_try = atan2(q_goal(2) - p_emit(2), q_goal(1) - p_emit(1));
            case RayType.specular
                if isempty(n_hat)
                    continue
                end
                th_try = reflectAngle(th_cur, n_hat);

            case RayType.diffuse
                if isempty(n_hat)
                    continue
                end
                th_try = wrapToPi(normal_world_angle + diffuse_offset);

            case RayType.random
                if isempty(n_hat)
                    th_try = wrapToPi(th_cur + (rand*2*pi - pi));
                else
                    th_spec = reflectAngle(th_cur, n_hat);
                    spread = pi/2;
                    th_try = wrapToPi(th_spec + (rand*2 - 1) * spread);
                end

            otherwise
                continue
        end

    % Collision checks using C-obstacles & convex boundary
        if ~robotInsideBoundsConvex([p_emit; th_cur], A, boundsE)
            continue
        end

        if ~configPointFreeCObs(p_emit, th_cur, A, B)
            continue
        end

        if ~rotateFreeCObs([p_emit; th_cur], th_try, A, B, boundsE, maxTurnPerStep)
            continue
        end

        backSeg = [hitPoint, p_emit];
        return
    end

    p_emit = [];
    backSeg = [];
    th_try = [];
end

% ---- Ray casting against polygons (goal/obstacles/bounds) ----
function [hitType, hitP, seg, n_hat] = castRay(p0, th, cObstacles, bounds, goalPoly, Lmax)

    u = [cos(th); sin(th)];
    seg = [];
    hitType = HitType.none;
    hitP = [];
    n_hat = [];

    % Build a long ray segment using your helpers
    Cray = fitSegment(p0, p0 + u*Lmax);

    % Goal poly
    tG = inf; pG = []; nG = [];
    if ~isempty(goalPoly)
        [tG, pG, nG] = firstHitWithPoly(Cray, p0, u, goalPoly);
    end

    % Obstacles (C-space polys)
    tO = inf; pO = []; nO = [];
    for k = 1:numel(cObstacles)
        [tk, pk, nk] = firstHitWithPoly(Cray, p0, u, cObstacles{k});
        if ~isnan(tk) && (tk < tO)
            tO = tk; pO = pk; nO = nk;
        end
    end

    % Boundary
    [tB, pB, nB] = firstHitWithPoly(Cray, p0, u, bounds);

    [tmin, idx] = min([tG, tO, tB, inf]);
    if isinf(tmin) || isnan(tmin)
        return
    end

    switch idx
        case 1, hitType = HitType.goal;     hitP = pG; n_hat = [];
        case 2, hitType = HitType.obstacle; hitP = pO; n_hat = nO;
        case 3, hitType = HitType.boundary; hitP = pB; n_hat = nB;
    end

    seg = [p0, hitP];

    if ~isempty(n_hat)
        if dot(n_hat, u) > 0
            n_hat = -n_hat;
        end
    end
end

function [tmin, Pmin, n_hat] = firstHitWithPoly(Cray, p0, u, V)
    tmin = inf;
    Pmin = [NaN; NaN];
    n_hat = [0;0];

    K = size(V,2);
    for i = 1:K
        a = V(:,i);
        b = V(:,mod(i,K)+1);

        Cedge = fitSegment(a, b);
        [hit, ~, xy] = intersectSegment(Cray, Cedge);

        if hit && ~isempty(xy)
            % multiple intersections possible on collinear; pick nearest
            for j = 1:size(xy,2)
                P = xy(:,j);
                t = dot(P - p0, u);  % distance along ray direction
                if (t > 1e-12) && (t < tmin)
                    tmin = t;
                    Pmin = P;
                    e = b - a;
                    n = [-e(2); e(1)];
                    n_hat = n / max(norm(n), eps);
                end
            end
        end
    end

    if isinf(tmin)
        tmin = NaN;
    end
end

function E = edgesFromVertices(V)
    K = size(V,2);
    v1 = V(:,1:K);
    v2 = V(:,[2:K,1]);
    e  = v2 - v1;
    n  = [-e(2,:); e(1,:)];
    E.v1 = v1;
    E.v2 = v2;
    E.e  = e;
    E.n  = n;
end

function inside = pointInConvex(p, V)
    K = size(V,2);
    x = p(1);
    y = p(2);
    prev = 0;
    for i = 1:K
        a = V(:,i);
        b = V(:,mod(i,K)+1);
        edge = b - a;
        rel  = [x - a(1); y - a(2)];
        crossz = edge(1)*rel(2) - edge(2)*rel(1);
        s = sign(crossz);
        if abs(s) < 1e-12
            s = 0;
        end
        if s ~= 0
            if prev == 0
                prev = s;
            elseif prev ~= s
                inside = false;
                return
            end
        end
    end
    inside = true;
end

function ok = robotInsideBoundsConvex(q, A, boundsE)
    R = [cos(q(3)) -sin(q(3)); sin(q(3)) cos(q(3))];
    A_trans = R*A + q(1:2);
    ok = true;
    for i = 1:size(A_trans,2)
        if ~pointInConvex(A_trans(:,i), boundsE.v1)
            ok = false;
            return
        end
    end
end

function ok = configPointFreeCObs(pXY, theta, A, B)
    for i = 1:numel(B)
        C = cObstacle(theta, A, B{i});
        if pointInConvex(pXY, C)
            ok = false;
            return
        end
    end
    ok = true;
end

function ok = rotateFreeCObs(q_from, theta_target, A, B, boundsE, maxTurnPerStep)
    dtheta  = wrapToPi(theta_target - q_from(3));
    nSteps  = max(2, ceil(abs(dtheta)/maxTurnPerStep));
    thetas  = q_from(3) + linspace(0, dtheta, nSteps);
    pXY     = q_from(1:2);
    ok = true;
    for i = 1:numel(thetas)
        th = thetas(i);
        if ~robotInsideBoundsConvex([pXY; th], A, boundsE)
            ok = false;
            return
        end
        if ~configPointFreeCObs(pXY, th, A, B)
            ok = false;
            return
        end
    end
end

function dmin = radialClearance(p, obstaclesB, bounds, nRays, Lmax)
    phis = linspace(0, 2*pi, nRays+1);
    phis(end) = [];
    dmin = inf;

    for idx = 1:numel(phis)
        phi = phis(idx);
        u = [cos(phi); sin(phi)];
        Cray = fitSegment(p, p + u*Lmax);

        d_ray = inf;

        % boundary
        Kb = size(bounds,2);
        for i = 1:Kb
            a = bounds(:,i);
            b = bounds(:,mod(i,Kb)+1);
            Cedge = fitSegment(a, b);
            [hit, ~, xy] = intersectSegment(Cray, Cedge);
            if hit && ~isempty(xy)
                for j = 1:size(xy,2)
                    t = dot(xy(:,j) - p, u);
                    if ~isnan(t) && (t >= 0)
                        d_ray = min(d_ray, t);
                    end
                end
            end
        end

        % obstacles
        for k = 1:numel(obstaclesB)
            V = obstaclesB{k};
            Ko = size(V,2);
            for i = 1:Ko
                a = V(:,i);
                b = V(:,mod(i,Ko)+1);
                Cedge = fitSegment(a, b);
                [hit, ~, xy] = intersectSegment(Cray, Cedge);
                if hit && ~isempty(xy)
                    for j = 1:size(xy,2)
                        t = dot(xy(:,j) - p, u);
                        if ~isnan(t) && (t >= 0)
                            d_ray = min(d_ray, t);
                        end
                    end
                end
            end
        end

        dmin = min(dmin, d_ray);
    end

    if isinf(dmin)
        dmin = 0;
    end
end

function th_r = reflectAngle(th_incident, n_hat)
    v = [cos(th_incident); sin(th_incident)];
    n = n_hat / max(norm(n_hat), eps);
    v_r = v - 2*(v.'*n)*n;
    th_r = atan2(v_r(2), v_r(1));
end

function ang = wrapToPi(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end

function debugPlotCspaceAtTheta(cObstacles)
    for i = 1:numel(cObstacles)
        C = cObstacles{i};
        [ph, lbl] = plotCObstacle(C, i);
        set(ph,  'Tag','CObs', 'HitTest','off');
        set(lbl, 'Tag','CObsLabel', 'HitTest','off');
    end
end

function debugStepBlock(doStep, pauseSec)
    if ~doStep
        return
    end
    if pauseSec > 0
        pause(pauseSec);
    else
        title('Collision: press any key / click to continue');
        drawnow limitrate;
        waitforbuttonpress;
    end
end
