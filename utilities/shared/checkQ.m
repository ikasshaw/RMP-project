% Isaac Shaw
% Robot Motion Planning
% 6/14/2025
% Function to check that a q configuration in 2d space is valid

function valid = checkQ(q)

    % Check q
    if ~isvector(q)
        error('q must be a 1-D vector')
    elseif numel(q) ~= 3
        error('q must be a 3x1 vector')
    elseif ~all(isreal(q), 'all')
        error('q must have only real values')
    elseif ~all(isfinite(q), 'all')
        error('q must have only finite values')
    end

    valid = true;

end