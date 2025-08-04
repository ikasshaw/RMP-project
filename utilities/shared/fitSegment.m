function C = fitSegment(v1, v2)

    % Check v1
    if ~isvector(v1)
        error('v1 must be a 1-D vector')
    elseif numel(v1) ~= 2
        error('v1 must be a 2x1 vector')
    elseif ~all(isreal(v1), 'all')
        error('v1 must have only real values')
    elseif ~all(isfinite(v1), 'all')
        error('v1 must have only finite values')
    end

    % Check v2
    if ~isvector(v2)
        error('v2 must be a 1-D vector')
    elseif numel(v2) ~= 2
        error('v2 must be a 2x1 vector')
    elseif ~all(isreal(v2), 'all')
        error('v2 must have only real values')
    elseif ~all(isfinite(v2), 'all')
        error('v2 must have only finite values')
    end

    S = [0, 1; 1, 1];
    C = [v1(1), v2(1); v1(2), v2(2)] * S^(-1);

end