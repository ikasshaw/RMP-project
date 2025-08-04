% Isaac Shaw
% Robot Motion Planning
% 6/14/2025

function [obstacle, label] = plotCObstacle(CB, i)

    % Check CB
    checkVertices(CB);

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

    obstacle = patch('Vertices', CB.', 'Faces', 1:length(CB), 'Parent', axs, 'EdgeColor', 'g', 'FaceColor', 'g', 'FaceAlpha', 0.1);
    
    ps = polyshape(CB(1,:),CB(2,:));
    [x,y] = centroid(ps);
    label = text(axs, x, y, sprintf('CB_%d', i));

    axis tight;

end