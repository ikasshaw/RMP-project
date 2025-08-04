% Isaac Shaw
% Robot Motion Planning
% 8/2/2025

function newA = moveRobot(q, A, options)

    arguments
        q double
        A double
        options.world logical = false 
    end

    % Check A
    checkVertices(A);

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

    newA = A;
    newA(3,:) = 0;
    newA(4,:) = 1;

    R = Rz(q(3));
    T = Tx(q(1))*Ty(q(2));

    if options.world
        h = T*R;
    else
        h = R*T;
    end

    newA = h * newA;

    newA = newA(1:2, :);
end