% Isaac Shaw
% Robot Motion Planning
% 6/14/2025
% Function to check the validity of a polygon with points defined by matrix M

function valid = checkVertices(M)

    [m, n] = size(M);
    
    % Basic checks
    if m ~= 2 % Check that we have 2 rows (1 for each axis)
        error('Matrix of vertices must be a 2xN matrix')
    elseif  n <= 2 % Check that we have at least 3 points 
        error('Matrix of vertices must have at least 3 points')
    elseif ~isnumeric(M) % Check that matrix is all numeric
            error('Matrix of vertices must be numeric')
    elseif ~isreal(M) % Check for all real
                error('Matrix of vertices must contain all real values')
    end
            
    % Check that points define convex hull in counter-clockwise order
    k = convhull(M');
    k(end) = [];
    
    if numel(k) ~= n

        error('Matrix must define the points of a convex polygon')
    end

    if ~isequal(k.',1:n)

        error('Matrix of vertices must define a convex polygon in ccw order')
    end 

    valid = true;

end