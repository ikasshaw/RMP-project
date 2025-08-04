% Isaac Shaw
% Robot Motion Planning
% 6/14/2025
% V2


function points = getConvexPolyPoints(n, options)

    % Generates the points for a convex polygon
    % This function generates the points by choosing angles from a central point and placing the points some radius away from that point
    % The default position of the polygons centroid is 0, 0 but can be specified with the centroid option
    % This function can generate polygons whose points are inscribed by a circle of the radius supplied
    % n defines the number of faces of the final polygon
    % Additional options are
    %   regular - Boolean to set whether the generated polygon should be a regular polygon or not
    %   centroid - 2x1 vector where the first element describes the x shift and the second the y shift
    %   radii - Scalar that defines the radius of the circle that inscribes the final polygon
    %   orientation - Pass 'r' for random or a scalar to set a specific angle from horizontal the first point of the object should be placed

    arguments
        n (1,1) = 3
        options.regular (1,1) = true
        options.centroid (2,1) double = [0,0]
        options.radius (1,1) double = 1
        % options.orientation (1,1) = 0
        options.orientation (1,1) = 'r'
    end
    
    % Input validation
    if n < 3
        error('Number of sides must be at least 3.');
    end

    if options.orientation == 'r'
        orientation = 2*pi*rand(1);
    elseif isnumeric(options.orientation)
        orientation = options.orientation;
    else
        error('orientation must either be numeric or r for random')
    end

    if ~isnumeric(options.radius)
        error('radius must be numeric')
    elseif ~isfinite(options.radius)
        error('radius must be finite')
    elseif ~isreal(options.radius)
        error('radius must be real')
    end
    
    % Generate the angles from the center of to the individual points. Equally spaced if equal set true, random otherwise
    if options.regular
        
        % define initial orientation at 0 and final orientation as 2 pi more than the initial
        orient_i = 0;
        orient_f = orient_i + (2 * pi);

        angles = linspace(orient_i, orient_f, n+1);
        angles(end) = []; % Remove the final angle
        
        % % Convert polar to cartesian coordinates
        x = options.radius .* cos(angles);
        y = options.radius .* sin(angles);

    % if not regular, generate random angles
    else
        angles = sort(rand(n,1) * 2 * pi)';
    end

    % % Convert polar to cartesian coordinates
    x = options.radius .* cos(angles);
    y = options.radius .* sin(angles);

    points = [x; y];
    ps = polyshape(points(1,:), points(2,:));
    [x, y] = centroid(ps);

    points(3,:) = 0;
    points(4,:) = 1;

    % if options.randorient
    %     orientation = 2*pi*rand(1);
    % else
    %     orientation = options.orientation;
    % end

    points = Rz(orientation)*points;

    points = Tx(x - options.centroid(1)) * Ty(y - options.centroid(2)) * points;
    
    points = points(1:2, :);

end

