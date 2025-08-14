function free = isConfigFree(q, A, obstacles_union, bounds, considerBounds)
    R = [cos(q(3)), -sin(q(3)); sin(q(3)), cos(q(3))];
    A_trans = R*A + q(1:2);
    robot_ps = polyshape(A_trans.', 'Simplify', false);

    if considerBounds && ~all(inpolygon(A_trans(1,:), A_trans(2,:), bounds(1,:), bounds(2,:)))
        free = false; return;
    end
    if overlaps(robot_ps, obstacles_union)
        free = false; return;
    end
    free = true;
end
