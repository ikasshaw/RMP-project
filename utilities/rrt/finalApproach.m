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

    xs = linspace(q_from(1), q_goal(1), nAng);
    ys = linspace(q_from(2), q_goal(2), nAng);
    seg2 = [xs; ys; repmat(alpha,1,nAng)];

    if ~rotateFree([q_goal(1); q_goal(2); alpha], q_goal(3), A, obstacles_union, bounds, considerBounds, nAng)
        return
    end

    seg3 = sampleRotation([q_goal(1); q_goal(2); alpha], q_goal(3), nAng);

    traj = [seg1, seg2(:,2:end), seg3(:,2:end)];

    ok = true;
end

% File size is blowing up...just put these helpers here
function seg = sampleRotation(q_from, theta_target, nAng)

    delta = wrapToPi(theta_target - q_from(3));
    thetas = q_from(3) + linspace(0, delta, nAng);

    seg = [repmat(q_from(1:2),1,numel(thetas)); thetas];
end

function ok = rotateFree(q_from, theta_target, A, obstacles_union, bounds, considerBounds, nSteps)

    delta = wrapToPi(theta_target - q_from(3));
    thetas = q_from(3) + linspace(0, delta, nSteps);

    ok = true;
    for i=1:numel(thetas)
        if ~isConfigFree([q_from(1); q_from(2); thetas(i)], A, obstacles_union, bounds, considerBounds)
            ok = false; return;
        end
    end
end