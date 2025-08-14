% Isaac Shaw
% Robot Motion Planning
% 8/10/2025

% THIS FUNCTION ASSUMES THE BOUNDARIES ARE THE MINIMUM BOUNDARIES FOR ALL ANGLES OF THE ROBOT
function qpath = rayCastPlanner(A, q_init, q_goal, B, bounds, options)

    arguments
        A (2,:) double
        q_init (3,1) double
        q_goal (3,1) double
        B cell

        % Assume the bounds are the absolute minimum bounding box of any robot orientation and robot frame at robot center
        bounds (2,:) double = [0; 0]
        
        options.rngSeed (1,1) double = 1

        options.initialRays (1,1) double = 5
        options.maxBounces (1,1) double {mustBeInteger,mustBeNonnegative} = 30
        options.maxIter (1,1) double = 5000
        options.minIter (1,1) double = 20
        
        options.goalRegionNRays (1,1) double = 36
        options.goalRegionMargin (1,1) double = 1e-3

        options.includeGoalReflection logical = true
        options.includeSpecularReflection logical = true
        options.includeDiffuseReflection logical = false
        options.includeRandomReflection logical = false
        options.numberOfDiffuse (1,1) double {mustBeInteger,mustBePositive} = 5

        options.turnPerStep (1,1) double = .2
        options.maxDistPerStep (1,1) double = .1
        options.scoreChildren logical = false
        
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
    
    rng(options.rngSeed);
    qpath = [];
    ZERO = 1e-8;

    childrenPerBounce = sum([options.includeGoalReflection, options.includeSpecularReflection, options.includeDiffuseReflection, options.includeRandomReflection, options.includeDiffuseReflection * options.numberOfDiffuse]);

    % Compute the radius of the robot as the length from the centroid to the farthest vertex
    [cx, cy] = centroid(polyshape(A.', 'Simplify', true));
    R_robot = max(sqrt((A(1,:) - cx).^2 + (A(2,:) - cy).^2));

    
    % Calculate worst case cObs over all angles (unless free flying then only calculate cObs for the initial config angle
    cObs = {};
    thetas = linspace(0, 2*pi, max(1, ceil(1/options.turnPerStep)));
    margin = R_robot - R_robot * cos(options.turnPerStep/2);
    for i = 1:numel(B)

        all_obs_cspace_verts = [];

        for t=1:numel(thetas)
            theta = thetas(t);
            all_obs_cspace_verts = [all_obs_cspace_verts, cObstacle(theta, A, B{i})];
        end

        % keep convex hull
        if size(all_obs_cspace_verts,2) >= 3
            
            % remove NaN ring separators and duplicate points
            all_obs_cspace_verts = rmmissing(all_obs_cspace_verts, 2);
            all_obs_cspace_verts = uniquetol(all_obs_cspace_verts.', 1e-3, 'ByRows', true).';
            k = convhull(all_obs_cspace_verts(1,:).', all_obs_cspace_verts(2,:).');
            k(end) = [];
            all_obs_cspace_verts(:,k);
        end

        % Inflate by margin (arc cover), then clean vertices
        all_obs_cspace_verts = polybuffer(polyshape(all_obs_cspace_verts.', 'Simplify', true), margin*2).Vertices.';

        % remove NaN ring separators and duplicate points
        all_obs_cspace_verts = rmmissing(all_obs_cspace_verts, 2);
        all_obs_cspace_verts = uniquetol(all_obs_cspace_verts.', 1e-4, 'ByRows', true).';

        % keep convex hull ring (no repeated last vertex)
        if size(all_obs_cspace_verts,2) >= 3
            k = convhull(all_obs_cspace_verts(1,:).', all_obs_cspace_verts(2,:).');
            k(end) = [];
            cObs{i} = all_obs_cspace_verts(:, k);
        else
            cObs{i} = all_obs_cspace_verts;
        end
    end

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

    % Debug bounds shape only
    boundsPS = polyshape(bounds.', 'Simplify', false);

    for k = 1:numel(cObs)

        if inpolygon(q_goal(1), q_goal(2), cObs{k}(1,:), cObs{k}(2,:))
            disp('Algorithm sees goal inside cObstacle and cannot solve.')
            return
        end
        if inpolygon(q_init(1), q_init(2), cObs{k}(1,:), cObs{k}(2,:))
            disp('Algorithm sees initial position inside cObstacle and cannot solve.')
            return
        end
    end

    % Compute a safe goal region to aim for
    % If this region is intersected by a ray, we know that we can safely reach the goal with a pivot drive pivot maneuver
    goalPoly = radialClearance(q_goal(1:2), cObs, bounds, options.goalRegionNRays, Lmax);

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

    StackItem = @(p,th,b,segs,rtype,htype,g) struct('p',p,'th',th,'bounces',b,'segs',segs,'rayType',rtype,'hitType',htype,'len',g);

    th0 = wrapToPi(q_init(3));
    th_goal = wrapToPi(atan2(q_goal(2) - q_init(2), q_goal(1) - q_init(1)));

    % Initial set of rays to cast and includes direct to goal, initial direction, and some random
    ths = wrapToPi(th0 + (2*pi)*rand(1,options.initialRays));
    dirs = [ths, th0, th_goal];

    % Preallocate a stack and track the index of the top element to save time with allocations...probably isn't saving much
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

    % Don't recompute for angle/xy pairs that are really close to something already tried.
    % Should stop casting from getting stuck in corner reflectionish cycles
    positionResolution = 0.02;
    angleResolution = deg2rad(1);
    visited = dictionary();
    makeKey = @(p,th,initP) keyHash([round(p(1)/positionResolution),...
                                     round(p(2)/positionResolution),...
                                     round(wrapToPi(th)/angleResolution),...
                                     round(initP(1)/positionResolution),...
                                     round(initP(2)/positionResolution)]);

    for i = 1:stackTop
        visited(makeKey(stack(i).p, stack(i).th, stack(i).segs(:,1))) = true;
    end

    iter = 0;

    if ~isempty(goalPoly) && inpolygon(q_init(1), q_init(2), goalPoly(1,:), goalPoly(2,:))
        disp('Trivial solution directly between q_init and q_goal found')

        heading = wrapToPi(atan2(q_goal(2) - q_init(2),  q_goal(1) - q_init(1)));
        successPaths{1} = [q_init, [q_goal(1:2); heading], q_goal];
        iter = options.maxIter;
        nSuccess = 1;
    end

    while iter < options.maxIter

        % Try another random starting angle if we dead end before min iters

        if (stackTop <= 0)
            if (iter <= options.minIter)
                stackTop = 1;
                stack(stackTop) = StackItem(q_init(1:2), rand*2*pi, 0, q_init, RayType.initial, HitType.none, 0);
                if options.debug
                    disp('Ran out of initial rays and children. Added new random ray')
                end
            else
                break;
            end
        end

        node  = stack(stackTop);
        stackTop = stackTop - 1;
        iter = iter + 1;

        p = node.p;
        th = wrapToPi(node.th);
        bounces = node.bounces;
        segs = node.segs;
        len = node.len;

        % Branch-and-bound lower bound
        h = max(0, norm(p - q_goal(1:2)));
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

            [hitType, hitPoint, segRay, hitNormal] = castRay(p, th, cObs, bounds, goalPoly, q_goal, Lmax);

            if isempty(hitPoint)
                continue
            elseif hitType ~= HitType.goal
                % Nudge the ray back just a bit to avoid going inside obstacles
                hitPoint = hitPoint - max(ZERO, ZERO*1e4*norm(segRay(:,2)-segRay(:,1))) * [cos(th); sin(th)];
            end

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
                                    options.includeRandomReflection);

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
            for k = numel(childRays):-1:1
                segs_i = [segs, [childRays(k).p_emit; childRays(k).th]];
                g_i = len + norm(childRays(k).p_emit - p);
                h_i = max(0, norm(childRays(k).p_emit - q_goal(1:2)));

                if (g_i + h_i) >= bestLen
                    continue
                end

                key = makeKey(childRays(k).p_emit, childRays(k).th, segs_i(1:2,1));
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
                    if options.debugPause > 0
                        pause(options.debugPause);
                    else
                        title('Collision: press any key / click to continue');
                        drawnow limitrate;
                        waitforbuttonpress;
                    end
                end
            end

            if stackTop > 0
                node  = stack(stackTop);
                stackTop = stackTop - 1;
                iter = iter + 1;
        
                p = node.p;
                th = wrapToPi(node.th);
                bounces = node.bounces;
                segs = node.segs;
                len = node.len;
            else
                break
            end

        end
    end

    if nSuccess == 0
        qpath = [];
        return
    end

    successPaths = successPaths(1:nSuccess);

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

    % Waypoints have the XY of the current waypoint and the heading to the next waypoint
    % so shift angles to previous waypoint and add heading from final waypoint to q_goal 
    waypoints = successPaths{minIdx};

    d = sqrt(sum(diff(waypoints,1,2).^2,1));

    qpath = [q_init];
    numTurnSteps = 40;
    % Waypoints (other than 1 (q_init) and end (q_goal) have the XY of the 
    % current waypoint and the heading to the next waypoint
    for idx = 1:size(waypoints,2) - 1
        
        % First turn to face next waypoint
        currentWaypoint = waypoints(:,idx);
        nextWaypoint = waypoints(:,idx+1);
            
        heading = wrapToPi(atan2( nextWaypoint(2) - currentWaypoint(2),  nextWaypoint(1) - currentWaypoint(1)))
            
        if abs(wrapToPi(currentWaypoint(3)) - heading) > ZERO * 100
            angleSteps = linspace(wrapToPi(currentWaypoint(3)), heading, numTurnSteps);
            angleSteps2 = linspace(wrapTo2Pi(currentWaypoint(3)), wrapTo2Pi(heading), numTurnSteps);
            if abs(angleSteps(2) - angleSteps(1)) > abs(angleSteps2(2) - angleSteps2(1))
                angleSteps = angleSteps2;
            end

            pivot = repmat(currentWaypoint, 1, numel(angleSteps));
            pivot(3,:) = angleSteps;
            qpath = [qpath, pivot];
        end
            
        if any(abs(currentWaypoint(1:2) - nextWaypoint(1:2)) > ZERO*100)
            numTravelSteps = ceil(d(idx)/options.maxDistPerStep);
            drive = [linspace(currentWaypoint(1), nextWaypoint(1), numTravelSteps); linspace(currentWaypoint(2), nextWaypoint(2), numTravelSteps)];
            drive(3,:) = heading;
            qpath = [qpath, drive];

        end

        if abs(wrapToPi(nextWaypoint(3)) - wrapToPi(currentWaypoint(3))) > ZERO
            angleSteps = linspace(wrapToPi(currentWaypoint(3)), wrapToPi(nextWaypoint(3)), numTurnSteps);
            angleSteps2 = linspace(wrapTo2Pi(currentWaypoint(3)), wrapTo2Pi(nextWaypoint(3)), numTurnSteps);
            if abs(angleSteps(2) - angleSteps(1)) > abs(angleSteps2(2) - angleSteps2(1))
                angleSteps = angleSteps2;
            end
            pivot = repmat(nextWaypoint, 1, numel(angleSteps));
            pivot(3,:) = angleSteps;
            qpath = [qpath, pivot];
        end
    end
end