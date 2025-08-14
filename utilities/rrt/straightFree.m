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