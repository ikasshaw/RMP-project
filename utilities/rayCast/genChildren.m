function childData = genChildren(hit_type, ...
                                 th_in, ...
                                 n_hat, ...
                                 parent_hit_point, ...
                                 q_goal, ...
                                 includeGoalReflection, ...
                                 includeSpecularReflection, ...
                                 includeDiffuseReflection, ...
                                 numberOfDiffuse, ...
                                 includeRandomReflection)

    u_ray = [cos(th_in); sin(th_in)];
    childData = struct('th',{},'p_emit',{},'backSeg',{},'rayType',{},'hitType',{});

    if ~isempty(n_hat)
        normal_world_angle = atan2(n_hat(2), n_hat(1));
    else
        normal_world_angle = NaN;
    end

    function try_add(angleType, diffuse_offset)

        switch angleType
            case RayType.goal
                th = atan2(q_goal(2) - parent_hit_point(2), q_goal(1) - parent_hit_point(1));
            case RayType.specular
                if isempty(n_hat)
                    return
                end
                th = reflectAngle(th_in, n_hat);

            case RayType.diffuse
                if isempty(n_hat)
                    return
                end
                th = wrapToPi(normal_world_angle + diffuse_offset);

            case RayType.random
                if isempty(n_hat)
                    th = wrapToPi(th_in + (rand*2*pi - pi));
                else
                    th_spec = reflectAngle(th_in, n_hat);
                    spread = pi/2;
                    th = wrapToPi(th_spec + (rand*2 - 1) * spread);
                end
        end
        if abs(wrapToPi(th - th_in)) > 1e-3
        % if ~isempty(p_emit) && ~isempty(backSeg) && abs(wrapToPi(th - th_in)) > 1e-3
            childData(end+1).th = th;
            childData(end).p_emit = parent_hit_point;
            % childData(end).backSeg = backSeg;
            childData(end).rayType = angleType;
            childData(end).hitType = hit_type;
        end
    end

    if includeGoalReflection
        try_add(RayType.goal, 0);
    end
    if ~isempty(n_hat)
        if includeSpecularReflection
            try_add(RayType.specular, 0);
        end

        if includeRandomReflection
            try_add(RayType.random, 0);
        end

        if includeDiffuseReflection
            diffuse_angles = linspace(-pi/2, pi/2, numberOfDiffuse);
            for kk = 1:numel(diffuse_angles)
                k = diffuse_angles(kk) + (rand*0.1 - 0.05);
                try_add(RayType.diffuse, k);
            end
        end
    end

end