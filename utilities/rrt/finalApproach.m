function [traj, ok] = finalApproach(q_from, q_goal, A, obstacles_union, bounds, considerBounds, nAng, nLine)

    traj = [];
    ok = false;
    alpha = atan2(q_goal(2)-q_from(2), q_goal(1)-q_from(1));

    if ~rotateFree(q_from, alpha, A, obstacles_union, bounds, considerBounds, nAng)
        return
    end

    seg1 = sampleRotation(q_from, alpha, nAng);

    if ~straightFree([q_from(1); q_from(2); alpha], q_goal(1:2), A, obstacles_union, bounds, considerBounds, nLine)
        return
    end

    xs = linspace(q_from(1), q_goal(1), nSteps);
    ys = linspace(q_from_(2), q_goal(2), nSteps);
    seg2 = [xs; ys; repmat(alpha,1,nSteps)];

    if ~rotateFree([q_goal(1); q_goal(2); alpha], q_goal(3), A, obstacles_union, bounds, considerBounds, nAng)
        return
    end

    seg3 = sampleRotation([q_goal(1); q_goal(2); alpha], q_goal(3), nAng);

    traj = [seg1, seg2(:,2:end), seg3(:,2:end)];

    ok = true;
end