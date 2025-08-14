function d = configDist(q1, q2, wTheta)
    dx = q1(1) - q2(1); dy = q1(2) - q2(2);
    dtheta = wrapToPi(q1(3) - q2(3));
    d = sqrt(dx^2 + dy^2) + wTheta * abs(dtheta);
end