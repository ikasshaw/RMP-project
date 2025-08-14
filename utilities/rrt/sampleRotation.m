function seg = sampleRotation(q_from, theta_target, nSteps)
    thetas = shortestSweep(q_from(3), theta_target, nSteps);
    seg = [repmat(q_from(1:2),1,numel(thetas)); thetas];
end