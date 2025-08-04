% Isaac Shaw
% Robot Motion Planning
% 6/14/2025

function [Fa, robot] = plotRobot(q, A, options)

    arguments
        q double 
        A double
        options.color (1,1) char = 'r';
        options.alpha (1,1) double {mustBeGreaterThanOrEqual(options.alpha, 0), mustBeLessThanOrEqual(options.alpha, 1)} = .5;
        options.text string = 'Robot'
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

    axs = gca;
    set(axs, 'NextPlot', 'add', 'DataAspectRatio', [1 1 1], 'Visible', 'on');

    R = Rz(q(3));
    T = Tx(q(1))*Ty(q(2));

    % If the translation is relative to the world coordinates, don't multiply them by the rotation matrix
    if options.world
        h = T*R;
    else
        h = R*T;
    end

    Fa = triad('Parent', axs, 'Matrix', h, 'AxisLabels', {'x_a', 'y_a', 'z_a'});
    
    robot = patch('Vertices', A.', 'Faces', 1:size(A,2), 'Parent', Fa, 'EdgeColor', options.color, 'FaceColor', options.color, 'FaceAlpha', options.alpha);
   
    ps = polyshape(A(1,:),A(2,:)); % polyshape object associated with A. (only used to get the centroid)
    [x,y] = centroid(ps); % x/y centroid of A
    label = text(axs, x, y, options.text, 'Parent', Fa);
    
    axis tight;
    
end