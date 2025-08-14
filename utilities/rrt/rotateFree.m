function ok = rotateFree(q_from, theta_target, A, obstacles_union, bounds, considerBounds, nSteps)

    delta = wrapToPi(theta_target, q_from);
    thetas = q_from + linespace(0, delta, nSteps);

    ok = true;
    for i=1:numel(thetas)
        if ~isConfigFree([q_from(1); q_from(2); thetas(i)], A, obstacles_union, bounds, considerBounds)
            ok = false; return;
        end
    end
end