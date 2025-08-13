function [hitType, hitP, seg, n_hat] = castRay(p0, th, cObstacles, bounds, goalPoly, q_goal, Lmax)

    u = [cos(th); sin(th)];
    seg = [];
    hitType = HitType.none;
    hitP = [];
    n_hat = [];

    % Build a long ray to use for the intersection checks
    Cray = fitSegment(p0, p0 + u*Lmax);

    % Goal poly
    tG = inf; pG = []; nG = [];
    if ~isempty(goalPoly)
        [tG, pG, nG] = firstHitWithPoly(Cray, p0, u, goalPoly);
    else
        % check if goal point lies on the ray segment
        z = Cray \ q_goal(1:2);
        s = z(1);
        if (abs(z(2) - 1) < 1e-8) && (s >= 0 && s <= 1)
            nG = [];
            pG = q_goal(1:2);
            tG = dot(q_goal(1:2) - p0, u);
        end
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