% Isaac Shaw
% Robot Motion Planning
% 8/10/2025


function qpath = rrtDiffDrive(A, q_init, q_goal, B, bounds, options)

    arguments
        A (2, :) double
        q_init (3,1) double
        q_goal (3,1) double
        B cell
        bounds (2, :) double = [0; 0] % Default bounds if not specified

        options.debug logical = false
        options.maxIter (1,1) double = 5000
        options.stepSize (1,1) double = 0.5
        options.goalBias (1,1) double = 0.1

        options.omegaMax (1,1) double = pi/2
        options.vMax (1,1) double = 1
        options.wTheta (1,1) double = 0.5

        options.numCollisionSteps (1,1) double = 20
        options.maxTurnPerStep (1,1) double = .1
        options.deltaT (1,1) double = 0.1
        options.considerBounds logical = true

        options.goalRegionMargin (1,1) double = 1e-3
        options.goalAngleSteps (1,1) double = 36
        options.goalLineSteps (1,1) double = 50
        options.goalRegionNRays (1,1) double = 360

        % options.steeringMode (1,:) char {mustBeMember(options.steeringMode, {'dubins','reeds-shepp'})} = 'dubins'
        options.steeringMode (1,:) char {mustBeMember(options.steeringMode, {'dubins','reeds-shepp'})} = 'reeds-shepp'
    end

    % Input checks
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

    % Obstacles
    obstacles = cell(1, numel(B));
    obstacles_union = polyshape;
    for i = 1:numel(B)
        obstacles{i} = polyshape(B{i}.', 'Simplify', false);
        obstacles_union = union(obstacles_union, obstacles{i});
    end
    boundsPolyshape = polyshape(bounds.', 'Simplify', false); % plotting only

    % Sampling limits
    x_lims = [min(bounds(1,:)), max(bounds(1,:))];
    y_lims = [min(bounds(2,:)), max(bounds(2,:))];

    % --- Max goal capture radius via ray casting (with rotation buffer) ---
    d_ray = radialClearance(q_goal(1:2), obstacles, bounds, options.goalRegionNRays);
    r_cap = max(0, d_ray - (R_robot + options.goalRegionMargin));

    % Init tree
    nodes(1) = struct('q', q_init, 'parent', 0, 'trajToParent', [], 'children', []);

    % Debug plotting
    if options.debug
        if isempty(groot().Children), figure('Name','rrtDiffDrive3d - DEBUG'); end
        hold on; axis equal;
        plot(boundsPolyshape, 'FaceColor','none','EdgeColor','k');
        for i = 1:numel(obstacles)
            plot(obstacles{i}, 'FaceColor','g','FaceAlpha',0.5);
        end
        plot(q_init(1), q_init(2), 'dg', 'MarkerSize',10, 'LineWidth',2);
        plot(q_goal(1), q_goal(2), 'dr', 'MarkerSize',10, 'LineWidth',2);
        if r_cap > 0
            th = linspace(0,2*pi,200);
            plot(q_goal(1)+r_cap*cos(th), q_goal(2)+r_cap*sin(th), 'm--', 'LineWidth',1.5);
        end
        drawnow;
    end

    foundPath = false;
    goalId = NaN;

    for iter = 1:options.maxIter
        % Sample
        if rand < options.goalBias
            q_rand = q_goal;
        else
            % Sampling step
            while true
                x = x_lims(1) + rand * diff(x_lims);
                y = y_lims(1) + rand * diff(y_lims);
                if options.considerBounds 
                    if inpolygon(x,y,bounds(1,:),bounds(2,:))
                        break
                    end
                else
                    break
                end
            end

            q_rand = [x; y; 2*pi*rand];

        end

        % Nearest
        [~, nearId] = min(arrayfun(@(n) configDist(n.q, q_rand, options.wTheta), nodes));
        q_near = nodes(nearId).q;

        % Steer with selected mode
        [q_new, traj, isFree] = steer(q_near, q_rand, options.stepSize, ...
                                      options.vMax, options.omegaMax, ...
                                      A, obstacles_union, options.numCollisionSteps, ...
                                      options.deltaT, bounds, options.considerBounds, ...
                                      options.steeringMode);
        if ~isFree || isempty(q_new), continue; end

        newId = numel(nodes) + 1;
        nodes(newId) = struct('q', q_new, 'parent', nearId, 'trajToParent', traj, 'children', []);
        nodes(nearId).children(end+1) = newId;

        if options.debug
            plot(traj(1,:), traj(2,:), 'b-', 'LineWidth', 1); drawnow ;
        end

        % Entered capture region? Do final approach.
        if r_cap > 0 && norm(q_new(1:2) - q_goal(1:2)) <= r_cap
            [traj_final, ok] = finalApproach(q_new, q_goal, A, obstacles_union, bounds, ...
                                 options.considerBounds, options.goalAngleSteps, options.goalLineSteps);
            if ok
                goalId = numel(nodes) + 1;
                nodes(goalId) = struct('q', q_goal, 'parent', newId, 'trajToParent', traj_final, 'children', []);
                foundPath = true;
                if options.debug
                    plot(traj_final(1,:), traj_final(2,:), 'r-', 'LineWidth', 2); drawnow;
                end
                break;
            end
        end

        % Fallback proximity attempt
        if configDist(q_new, q_goal, options.wTheta) < options.stepSize
            [q_new_goal, traj_goal, isFree_goal] = steer(q_new, q_goal, options.stepSize, ...
                                              options.vMax, options.omegaMax, ...
                                              A, obstacles_union, options.numCollisionSteps, ...
                                              options.deltaT, bounds, options.considerBounds, ...
                                              options.steeringMode);
            if isFree_goal && ~isempty(q_new_goal)
                goalId = numel(nodes) + 1;
                nodes(goalId) = struct('q', q_goal, 'parent', newId, 'trajToParent', traj_goal, 'children', []);
                foundPath = true;
                if options.debug
                    plot(traj_goal(1,:), traj_goal(2,:), 'r-', 'LineWidth', 2); drawnow;
                end
                break;
            end
        end
    end

    % Extract path
    if ~foundPath
        qpath = []; return;
    end

    idxs = goalId;
    cur = nodes(goalId).parent;
    while cur ~= 0
        idxs(end+1) = cur; %#ok<AGROW>
        cur = nodes(cur).parent;
    end
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

% Distance & geometry helpers
function dmin = radialClearance(p, obstacles, bounds, nRays)
    phis = linspace(0, 2*pi, nRays+1); phis(end) = [];
    dmin = inf;
    polys = [obstacles, {polyshape(bounds.', 'Simplify', false)}];
    for phi = phis
        u = [cos(phi); sin(phi)];
        d_ray = inf;
        for k = 1:numel(polys)
            V = polys{k}.Vertices.'; % 2xK
            K = size(V,2);
            for i = 1:K
                a = V(:,i);
                b = V(:,mod(i,K)+1);
                t = raySegIntersectDist(p, u, a, b);
                if ~isnan(t)
                    d_ray = min(d_ray, t);
                end
            end
        end
        dmin = min(dmin, d_ray);
    end
    if isinf(dmin), dmin = 0; end
end

function t = raySegIntersectDist(p, u, a, b)
    v = b - a;
    M = [u, -v];
    rhs = a - p;
    detM = u(1)*(-v(2)) - u(2)*(-v(1));
    if abs(detM) < 1e-12, t = NaN; return; end
    sol = M \ rhs;  t = sol(1); s = sol(2);
    if t >= 0 && s >= 0 && s <= 1
        if t < 1e-12, t = NaN; end
    else
        t = NaN;
    end
end

% === RRT helpers ===
function d = configDist(q1, q2, wTheta)
    dx = q1(1) - q2(1); dy = q1(2) - q2(2);
    dtheta = wrapToPi(q1(3) - q2(3));
    d = sqrt(dx^2 + dy^2) + wTheta * abs(dtheta);
end

function [q_new, traj, isFree] = steer(q_from, q_to, stepSize, vMax, omegaMax, ...
                                       A, obstacles_union, numSteps, ~, ...
                                       bounds, considerBounds, steeringMode)

    % Choose controls per steering mode (local Reeds–Shepp vs. Dubins)
    dx = q_to(1)-q_from(1); dy = q_to(2)-q_from(2);
    d = hypot(dx,dy);
    alpha = atan2(dy,dx);
    e = wrapToPi(alpha - q_from(3));  % heading error

    if d < 1e-6
        % Orientation-only target
        dtheta = wrapToPi(q_to(3)-q_from(3));
        if abs(dtheta) < 1e-6
            q_new = q_from; traj = q_from; isFree = true; return;
        end
        omega = sign(dtheta)*omegaMax; v = 0; t = abs(dtheta)/max(abs(omega),eps);
    else
        switch steeringMode
            case 'reeds-shepp'
                % May reverse if goal is "behind"
                if cos(e) >= 0
                    v = vMax;
                else
                    v = -vMax; alpha = wrapToPi(alpha + pi); e = wrapToPi(alpha - q_from(3));
                end
            case 'dubins'
                % Forward-only. If goal behind, rotate in place to reduce |e|.
                if abs(e) > pi/2
                    v = 0; omega = omegaMax * sign(e);
                    t = (abs(e) - pi/2) / max(abs(omega), eps);
                    if t <= 0, t = min(abs(e)/max(abs(omega),eps),  stepSize/max(vMax,eps)); end
                    % Simulate pure rotation below
                else
                    v = vMax;
                end
        end
        if abs(v) > 0
            omega = omegaMax * (e / (pi/2));   % saturated turn toward target
            t = stepSize / max(abs(v),eps);    % advance a fixed arc length
        else
            % pure rotation time already set for Dubins when needed
            if ~exist('t','var'), t = min(abs(e)/max(abs(omega),eps), 1.0); end
        end
    end

    % Propagate closed-form
    if abs(v) < 1e-6 && abs(omega) > 1e-6
        q_new = [q_from(1:2); q_from(3)+omega*t];
    elseif abs(omega) < 1e-6
        q_new = [q_from(1)+v*t*cos(q_from(3)); q_from(2)+v*t*sin(q_from(3)); q_from(3)];
    else
        q_new_theta = q_from(3)+omega*t;
        q_new_x = q_from(1) + (v/omega)*(sin(q_new_theta)-sin(q_from(3)));
        q_new_y = q_from(2) + (v/omega)*(cos(q_from(3))-cos(q_new_theta));
        q_new = [q_new_x; q_new_y; q_new_theta];
    end

    % Discretize & collision-check
    isFree = true;
    traj = zeros(3, numSteps+1); traj(:,1) = q_from;
    for k = 1:numSteps
        tk = k/numSteps * t;
        if abs(v) < 1e-6 && abs(omega) > 1e-6
            x = q_from(1); y = q_from(2); th = q_from(3)+omega*tk;
        elseif abs(omega) < 1e-6
            x = q_from(1)+v*tk*cos(q_from(3));
            y = q_from(2)+v*tk*sin(q_from(3)); th = q_from(3);
        else
            th = q_from(3)+omega*tk;
            x = q_from(1) + (v/omega)*(sin(th)-sin(q_from(3)));
            y = q_from(2) + (v/omega)*(cos(q_from(3))-cos(th));
        end
        q_k = [x; y; th];
        traj(:,k+1) = q_k;
        if ~isConfigFree(q_k, A, obstacles_union, bounds, considerBounds)
            isFree = false; q_new = []; traj = []; break;
        end
    end
end


% === Final approach & micro-snap (with shortest-angle rotations) ===
function [traj, ok] = finalApproach(q_from, q_goal, A, obstacles_union, bounds, considerBounds, nAng, nLine)
    traj = []; ok = false;
    % 1) rotate to face goal (shortest sweep)
    alpha = atan2(q_goal(2)-q_from(2), q_goal(1)-q_from(1));
    if ~rotateFree(q_from, alpha, A, obstacles_union, bounds, considerBounds, nAng), return; end
    seg1 = sampleRotation(q_from, alpha, nAng);
    % 2) straight to goal position
    if ~straightFree([q_from(1); q_from(2); alpha], q_goal(1:2), A, obstacles_union, bounds, considerBounds, nLine), return; end
    seg2 = sampleStraight([q_from(1); q_from(2); alpha], q_goal(1:2), nLine);
    % 3) rotate to goal orientation (shortest sweep)
    if ~rotateFree([q_goal(1); q_goal(2); alpha], q_goal(3), A, obstacles_union, bounds, considerBounds, nAng), return; end
    seg3 = sampleRotation([q_goal(1); q_goal(2); alpha], q_goal(3), nAng);
    traj = [seg1, seg2(:,2:end), seg3(:,2:end)];
    ok = true;
end


% --- Shortest-angle rotation helpers ---
function thetas = shortestSweep(theta0, theta1, nSteps)
    delta = wrapToPi(theta1 - theta0);
    thetas = theta0 + linspace(0, delta, nSteps);
end


% Check for collisions over a discrete range of angles
function ok = rotateFree(q_from, theta_target, A, obstacles_union, bounds, considerBounds, nSteps)
    thetas = shortestSweep(q_from(3), theta_target, nSteps);
    ok = true;
    for i=1:numel(thetas)
        if ~isConfigFree([q_from(1); q_from(2); thetas(i)], A, obstacles_union, bounds, considerBounds)
            ok = false; return;
        end
    end
end

function seg = sampleRotation(q_from, theta_target, nSteps)
    thetas = shortestSweep(q_from(3), theta_target, nSteps);
    seg = [repmat(q_from(1:2),1,numel(thetas)); thetas];
end

function ok = straightFree(q_from_aligned, xy_goal, A, obstacles_union, bounds, considerBounds, nSteps)
    xs = linspace(q_from_aligned(1), xy_goal(1), nSteps);
    ys = linspace(q_from_aligned(2), xy_goal(2), nSteps);
    th = q_from_aligned(3);
    ok = true;
    for i=1:nSteps
        if ~isConfigFree([xs(i); ys(i); th], A, obstacles_union, bounds, considerBounds)
            ok = false; return;
        end
    end
end


function seg = sampleStraight(q_from_aligned, xy_goal, nSteps)
    xs = linspace(q_from_aligned(1), xy_goal(1), nSteps);
    ys = linspace(q_from_aligned(2), xy_goal(2), nSteps);
    seg = [xs; ys; repmat(q_from_aligned(3),1,nSteps)];
end

% Check that a given configuration has no collisions
function free = isConfigFree(q, A, obstacles_union, bounds, considerBounds)
    R = [cos(q(3)), -sin(q(3)); sin(q(3)), cos(q(3))];
    A_trans = R*A + q(1:2);
    robot_ps = polyshape(A_trans.', 'Simplify', false);

    if considerBounds
        if ~all(inpolygon(A_trans(1,:), A_trans(2,:), bounds(1,:), bounds(2,:)))
            free = false; return;
        end
    end
    if overlaps(robot_ps, obstacles_union)
        free = false; return;
    end
    free = true;
end