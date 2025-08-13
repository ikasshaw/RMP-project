function th_r = reflectAngle(th_incident, n_hat)
    v = [cos(th_incident); sin(th_incident)];
    n = n_hat / max(norm(n_hat), eps);
    v_r = v - 2*(v.'*n)*n;
    th_r = atan2(v_r(2), v_r(1));
end