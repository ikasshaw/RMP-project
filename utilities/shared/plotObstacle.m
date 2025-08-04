% Isaac Shaw
% Robot Motion Planning
% 6/14/2025

function [obstacle, label] = plotObstacle(B, i)

    % Check B
    checkVertices(B);

    % Check i
    if numel(i) ~= 1
        error('i must be 1x1')
    elseif i <= 0
        error('i must be positive and non-zero')
    elseif ~all(isreal(i), 'all')
        error('i must real')
    elseif ~all(isfinite(i), 'all')
        error('i must be finite')
    end

    axs = gca;
    set(axs, 'NextPlot', 'add', 'DataAspectRatio', [1 1 1], 'Visible', 'on');

    obstacle = patch('Vertices', B.', 'Faces', 1:length(B), 'Parent', axs, 'EdgeColor', 'b', 'FaceColor', 'b', 'FaceAlpha', 0.1);
    
    ps = polyshape(B(1,:),B(2,:)); % polyshape object associated with B. (only used to get the centroid)
    [x,y] = centroid(ps); % x/y centroid of B
    label = text(axs, x, y, sprintf('B_{%d}', i));

    axis tight;

end