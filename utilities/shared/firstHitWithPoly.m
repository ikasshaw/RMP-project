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