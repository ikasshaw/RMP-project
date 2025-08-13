function dmin = radialClearance(p, cObstacles, bounds, nRays, Lmax)
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
        for k = 1:numel(cObstacles)
            V = cObstacles{k};
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