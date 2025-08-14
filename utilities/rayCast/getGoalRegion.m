function goalRegion = radialClearance(p, cObstacles, bounds, nRays, Lmax)

    % Use of this function in rayCast assumes the cObstacles are the cObstacles projected on the plane across all robot configurations
    % Also assumes the bounds represent the minimum bounds for the robot again in C-space

    goalRegion = [];
    phis = linspace(0, 2*pi, nRays+1);
    dmin = inf;
    % Get the shortest collision length in c-Space
    for idx = 1:numel(phis)
        phi = phis(idx);
        u = [cos(phi); sin(phi)];
        Cray = fitSegment(p, p + u*Lmax);

        % Obstacles (C-space polys)
        
        min_obs_len = inf;
        for k = 1:numel(cObstacles)
            [min_obs_k_len, ~, ~] = firstHitWithPoly(Cray, p, u, cObstacles{k});
            if ~isnan(min_obs_k_len) && (min_obs_k_len < min_obs_len)
                min_obs_len = min_obs_k_len;
            end
        end

        % Boundary
        [min_bnds_len, ~, ~] = firstHitWithPoly(Cray, p, u, bounds);

        d_ray = min([min_obs_len, min_bnds_len, inf]);

        dmin = min(dmin, d_ray);
    end

    if isinf(dmin)
        return
    end

    % attempt to create a polygon with nRays rays.
    % Check if this polygon lies in the bounds and freespace
    % if so, return. Otherwise, reduce size by 75 percent and try (again up to 5 times)
    for i =1:5
        verts = p + [cos(phis); sin(phis)] * dmin;
        ps = polyshape(verts.');
        inBounds = true;
        inFree = true;

        for j =1:size(verts, 2)
            if ~all(inpolygon(verts(1,:), verts(2,:), bounds(1,:), bounds(2,:)))
                inBounds = false;
                break
            end
        end

        if inBounds
            for k = 1:numel(cObstacles)
                ob_ps = polyshape(cObstacles{k}.');
                int = intersect(ps, ob_ps);
                if int.NumRegions > 0
                    inFree = false;
                    break
                end
            end
        end

        if inFree
            goalRegion = verts;
            return
        else
            dmin = dmin * .75;
        end
    end
end