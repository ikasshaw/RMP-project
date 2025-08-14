function [q_new, traj, isFree] = steer(q_from, q_to, stepSize, vMax, omegaMax, ...
                                       A, obstacles_union, numSteps, ~, ...
                                       bounds, considerBounds, steeringMode)

    dx = q_to(1)-q_from(1); dy = q_to(2)-q_from(2);
    d = hypot(dx,dy);
    alpha = atan2(dy,dx);
    e = wrapToPi(alpha - q_from(3));  % heading error

    if d < 1e-6
        % Orientation-only target
        dtheta = wrapToPi(q_to(3)-q_from(3));
        if abs(dtheta) < 1e-6
            q_new = q_from; traj = q_from; isFree = true; return;
        end
        omega = sign(dtheta)*omegaMax; v = 0; t = abs(dtheta)/max(abs(omega),eps);
    else
        switch steeringMode
            case 'reeds-shepp'  % allow reverse
                if cos(e) >= 0, v =  vMax; else, v = -vMax; alpha = wrapToPi(alpha + pi); e = wrapToPi(alpha - q_from(3)); end
            case 'dubins'       % forward-only
                if abs(e) > pi/2
                    v = 0; omega = omegaMax * sign(e);
                    t = (abs(e) - pi/2) / max(abs(omega), eps);
                    if t <= 0, t = min(abs(e)/max(abs(omega),eps),  stepSize/max(vMax,eps)); end
                else
                    v = vMax;
                end
        end
        if abs(v) > 0
            omega = omegaMax * (e / (pi/2));                 % saturating turn toward target
            t = stepSize / max(abs(v),eps);                   % fixed arc length
        elseif ~exist('t','var')
            t = min(abs(e)/max(abs(omega),eps), 1.0);         % pure pivot duration
        end
    end

    % Propagate 
    if abs(v) < 1e-6 && abs(omega) > 1e-6
        q_new = [q_from(1:2); q_from(3)+omega*t];
    elseif abs(omega) < 1e-6
        q_new = [q_from(1)+v*t*cos(q_from(3)); q_from(2)+v*t*sin(q_from(3)); q_from(3)];
    else
        thn = q_from(3)+omega*t;
        q_new = [ q_from(1) + (v/omega)*(sin(thn)-sin(q_from(3))); ...
                  q_from(2) + (v/omega)*(cos(q_from(3))-cos(thn)); ...
                  thn ];
    end

    % Discretize and collision check against exact robot geometry
    isFree = true;
    traj = zeros(3, numSteps+1); traj(:,1) = q_from;
    for k = 1:numSteps
        tk = k/numSteps * t;
        if abs(v) < 1e-6 && abs(omega) > 1e-6
            x = q_from(1); y = q_from(2); th = q_from(3)+omega*tk;
        elseif abs(omega) < 1e-6
            x = q_from(1)+v*tk*cos(q_from(3));
            y = q_from(2)+v*tk*sin(q_from(3)); th = q_from(3);
        else
            th = q_from(3)+omega*tk;
            x = q_from(1) + (v/omega)*(sin(th)-sin(q_from(3)));
            y = q_from(2) + (v/omega)*(cos(q_from(3))-cos(th));
        end
        q_k = [x; y; th];
        traj(:,k+1) = q_k;
        if ~isConfigFree(q_k, A, obstacles_union, bounds, considerBounds)
            isFree = false; q_new = []; traj = []; break;
        end
    end
end