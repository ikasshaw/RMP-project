% Isaac Shaw
% Robot Motion Planning
% 7/26/2025

function [adj, wAdj, xy] = approxCellGraph(q_init, q_goal, CB, bounds, options)

    arguments
        q_init double
        q_goal double
        CB cell
        bounds double
        options.debug logical = false
        options.maxDepth double = 10
        options.minRectScale double = 20
        options.brute logical = false
        options.dtheta (1,1) double = .01
    end

    % Input variable checks
    checkQ(q_init);
    checkQ(q_goal);
    checkVertices(bounds);

    % Check that all points are within the bounds
    if ~inpolygon(q_init(1), q_init(2), bounds(1,:), bounds(2,:))
        error('All points must be inside the boundary')
    elseif ~inpolygon(q_goal(1), q_goal(2), bounds(1,:), bounds(2,:))
        error('All points must be inside the boundary')
    end

    if ~iscell(CB)
        error('CB must be a cell array')
    else
        for i=1:numel(CB)
            checkVertices(CB{i});
            if ~inpolygon(CB{i}(1,:), CB{i}(2,:), bounds(1,:), bounds(2,:));
                error('All points must be inside the boundary')
            end
        end
    end

    if options.debug
        if isempty(groot().Children)
            fig = figure('Name', 'approxCellGraph - DEBUG');
        else
            fig = gcf;
        end
        axs = axes('Parent', fig, 'NextPlot', 'add', 'DataAspectRatio', [1,1,1]);

        for i = 1:numel(CB)
            ptc_CB(i) = plotCObstacle(CB{i}, i);
        end
        
        plt_init = plot(axs, q_init(1), q_init(2), 'og');
        plt_goal = plot(axs, q_goal(1), q_goal(2), 'xr');
        
        ptc_Bnds = patch('Vertices', bounds.', 'Faces', 1:size(bounds, 2), 'Parent', axs, 'EdgeColor', 'k', 'FaceColor', 'None', 'LineWidth', 1.5);
    end

    verts = [q_init(1:2,:), q_goal(1:2,:), bounds, horzcat(CB{:})];

    vertices_x_lims = [min(verts(1,:)), max(verts(1,:))];
    vertices_y_lims = [min(verts(2,:)), max(verts(2,:))];

    % Defines CCW rectangle points starting with lower left as initial point
    initialRectVerts = [vertices_x_lims(1), vertices_x_lims(2), vertices_x_lims(2), vertices_x_lims(1);...
        vertices_y_lims(1), vertices_y_lims(1), vertices_y_lims(2), vertices_y_lims(2)];

    R = struct('vertices', initialRectVerts,...
        'subdivide', true,...
        'state', 'Mixed',...
        'ps',polyshape(initialRectVerts(1,:), initialRectVerts(2,:)),...
        'id', 1,...
        'depth', 0);

    % Initialize a stack to be used for the iterative subdividing
    stack = {};
    stack{end+1} = R;
    empty_rects = {};
    nodeCount = 1;
    minRectDiag = norm(diff(R.vertices(:, [1,3]),1,2))/options.minRectScale;

    % Define a total polyshape to simplify the overlap checks
    obsUnion = polyshape;
    for i = 1:numel(CB)
        ps_CB(i) = polyshape(CB{i}(1,:), CB{i}(2,:));
        obsUnion = union(obsUnion, ps_CB(i));
    end

    if options.debug
        ptc = {};
        ptc{1} = patch('Vertices', R.vertices.', 'Faces', 1:size(R.vertices, 2), 'Parent', axs, 'EdgeColor', 'r', 'FaceColor', 'r', 'FaceAlpha', .1);
    end

    %% Decompose Cells
    while ~isempty(stack)
        
        curRect = stack{1};
        stack(1) = [];
        
        % Subdivide and process children in determined mixed when created
        if curRect.subdivide
            subdivided_R = subDivRec(curRect.vertices);
            curRect.subdivided = true;

            % Returns if either max depth or min rectangle size reached
            if curRect.depth >= options.maxDepth || minRectDiag > norm(diff(subdivided_R{1}(:, [1,3]), 1,2))
                disp('Reached minimum rectangle size!')
                break
            end

            if options.debug
                delete(ptc{curRect.id});
                ptc{curRect.id} = [];
            end

            for child_idx=1:4
                child.vertices = subdivided_R{child_idx};
                child.id = nodeCount + child_idx;
                child.ps = polyshape(subdivided_R{child_idx}(1,:), subdivided_R{child_idx}(2,:));
                child.state = classifyIntersection(child.ps, obsUnion);
                child.depth = curRect.depth + 1;
                child.subdivide = strcmp(child.state, 'Mixed');

                stack = [stack, {child}];

                if strcmp(child.state, 'Empty')
                    empty_rects{end+1} = child;
                end
                
                if options.debug
                    if strcmp(child.state, 'Mixed')
                        color = 'm';
                        edgeColor = 'None';
                        opacity = .5;
                    elseif strcmp(child.state, 'Full')
                        color = 'r';
                        edgeColor = 'None';
                        opacity = .75;
                    elseif strcmp(child.state, 'Empty')
                        color = 'c';
                        edgeColor = 'c';
                        opacity = .5;
                    end
                    
                    ptc{child.id} = patch('Vertices', child.vertices.', 'Faces', 1:size(child.vertices, 2), 'Parent', axs, 'EdgeColor', edgeColor, 'FaceColor', color, 'FaceAlpha', opacity);
                end
                
            end

            nodeCount = nodeCount + 4;

            % Draw after each subdivide, not for each child
            if options.debug
                drawnow;
            end
        end
    end

    % Precompute all object line segments
    Obs_segments = cell(numel(CB)*2);
    for idx = 1:numel(CB)
        cb = CB{idx};
        cbN = size(CB{idx}, 2);
        cbidx_segments = zeros(2, cbN*2);
        for vertex = 1:size(cb, 2)
            if vertex == cbN
                nextVertex = 1;
            else
                nextVertex = vertex + 1;
            end
            C_obs = fitSegment(cb(:,vertex), cb(:,nextVertex));
            cbidx_segments(:, vertex*2-1:vertex*2) = C_obs;
        end
        Obs_segments{idx} = cbidx_segments;
    end

    n = numel(empty_rects) + 2; % Each empty cell plus q_init and q_goal
    adj = zeros(n, n);
    wAdj = zeros(n, n); % Done to match outputs from the provided test inputs...should be inf
    % wAdj = inf(n, n);

    %% Begin adjacency matrix generation
    if ~options.brute
        % Only generate this initial xy list for plotting the diamonds in debug mode
        if options.debug
            xyDebug(:,1) = q_init(1:2,1);
            for j = 1:numel(empty_rects)
                rect = empty_rects{j};
                [x,y] = centroid(rect.ps);
                xyDebug(:,j+1) = [x;y];
            end
            xyDebug(:,end + 1) = q_goal(1:2,1);
            plt_diamonds = plot(axs, xyDebug(1,:), xyDebug(2,:), 'dk');
            plt = plot(axs, [xyDebug(1,1), xyDebug(1,2)], [xyDebug(2,1), xyDebug(2,2)], 'r');
        end
        
        %% Non-Brute Force Begin
        % For inner loop, only check elements that haven't been A yet. Because the graph is undirected, don't need to look at both A->B and B->A
        % and omits q_init and q_goal
        toCheckIdx = 1:n-2;
        xy(:,1) = q_init(1:2,1);
        % Main loops to check for valid segments between each pair of vertices
        for idxA=1:n-2

            rectA = empty_rects{idxA};
            [rectA_X, rectA_Y ]= centroid(rectA.ps);
            rectACentroid = [rectA_X; rectA_Y];
            xy(:, idxA + 1) = rectACentroid;

            % Checks for q_init and goal. These should only connect with the cell they are contained in
            if isinterior(rectA.ps, q_init(1), q_init(2))
                adj(1,idxA+1) = 1;
                wAdj(1, idxA+1) = norm(q_init(1:2,1) - rectACentroid);
                if options.debug
                    plot(axs,[q_init(1), rectACentroid(1)],[q_init(2), rectACentroid(2)], 'm', 'Tag', 'DEBUG', 'Linewidth', .5);
                    drawnow;
                end
            end

            if isinterior(rectA.ps, q_goal(1), q_goal(2))
                adj(idxA+1, n) = 1;
                wAdj(idxA+1, n) = norm(q_goal(1:2,1) - rectACentroid);
                if options.debug
                    plot(axs,[q_goal(1), rectACentroid(1)],[q_goal(2), rectACentroid(2)], 'g', 'Tag', 'DEBUG', 'Linewidth', 1.5);
                    drawnow;
                end
            end

            % for idxB=toCheckIdx
            for idxB=[idxA:n-2]

                if idxA == idxB
                    wAdj(idxA+1, idxB+1) = 0;
                    continue
                end

                rectB = empty_rects{idxB};
                [rectB_X, rectB_Y ]= centroid(rectB.ps);
                rectBCentroid = [rectB_X; rectB_Y];
                centroids = [rectACentroid, rectBCentroid];

                % check that cells are adjacent in lieu of actual book-keeping...
                touch = checkAdjacent(rectA, rectB);

                if ~touch
                    continue
                end

                if options.debug
                    set(plt, 'XData', centroids(1, :), 'YData', centroids(2, :));
                    drawnow;
                end
                % You don't need to check for line intersections if the cells are empty and truly adjacent. The line between centroids always passes through the overlapping edge.

                % Using the following wolfram code and assuming worst case with the small cell ll-corner placed at the origin and the larger cell placed on top with a scale between rectangles of c
                        % x1 = 1/2*w;
                        % x2 = 1/2*w*c;
                        % y1 = 1/2*h;
                        % y2 = h + 1/2*h*c;
                        % m = (y2 - y1)/ (x2 - x1);
                        % b = 1/2*h - (1/2*w) * m;
                        % x = (h - b)/m;
                        % Simplify[x] 

                        % OUTPUTS >> wc/(c+1)

                % Following this through, the x location of the intersection must always be < w. Affine transforms allow you to set this same scenario up for any pair of worst case cells. (this breaks down if the ratio = 1 because then the slope is infinity...

                % Cs = fitSegment(centroids(:, 1), centroids(:, 2));
                % hit = false;
                
                % for idx = 1:numel(CB)
                %     cb = CB{idx};
                %     [~, cbN] = size(cb);
                %     cur_obs_segments = Obs_segments{idx};
                %     for edgeIdx = 1:cbN
                %         Co = cur_obs_segments(:, edgeIdx*2-1:edgeIdx*2);
                %         [intersects, ~ , ~] = intersectSegment(Co, Cs);
                %         if intersects; hit = true; break; end
                %     end
                %     if hit; break; end
                % end
                
                % if hit; continue; end
                adj(idxA+1, idxB+1) = 1;
                wAdj(idxA+1, idxB+1) = norm(centroids(:, 1) - centroids(:, 2));
                if options.debug
                    plot(axs,centroids(1, :), centroids(2, :), 'm', 'Tag', 'DEBUG', 'Linewidth', .5);
                    drawnow;
                end
            end
        end
        xy(:,end + 1) = q_goal(1:2,1);
        % replace lower triangle to make adjacency matrices symmetric
        wAdj = fillmissing((wAdj-tril(wAdj)), 'constant', 0) + triu(wAdj).';
        adj = (adj-tril(adj))+triu(adj).';

    else 
        %% Brute Force Begin
        xy(:,1) = q_init(1:2,1);
        for j = 1:numel(empty_rects)
            rect = empty_rects{j};
            [x,y] = centroid(rect.ps);
            xy(:,j+1) = [x;y];
        end
        xy(:,end + 1) = q_goal(1:2,1);

        if options.debug
            plt_diamonds = plot(axs, xy(1,:), xy(2,:), 'dk');
            plt = plot(axs, [xy(1,1), xy(1,2)], [xy(2,1), xy(2,2)], 'r');
        end

        % Main loops to check for valid segments between each pair of vertices
        % Undirected adjacency matrices are symmetric so only iterate over
        % lower triangle
        for vertexA_idx=1:n
            for vertexB_idx =1:vertexA_idx
                if vertexA_idx == vertexB_idx
                    wAdj(vertexA_idx, vertexB_idx) = 0;
                    continue
                end
                
                if options.debug
                    set(plt, 'XData', [xy(1,vertexA_idx), xy(1,vertexB_idx)], 'YData', [xy(2,vertexA_idx), xy(2,vertexB_idx)]);
                    drawnow;
                end
                
                Cs = fitSegment(xy(:, vertexA_idx), xy(:, vertexB_idx));
                hit = false;
                
                for idx = 1:numel(CB)
                    cb = CB{idx};
                    [~, cbN] = size(cb);
                    cur_obs_segments = Obs_segments{idx};
                    for edgeIdx = 1:cbN
                        
                        Co = cur_obs_segments(:, edgeIdx*2-1:edgeIdx*2);
                        [intersects, ~, ~] = intersectSegment(Co, Cs);
                        if intersects; hit = true; break; end
                        
                    end
                    if hit; break; end
                end
                
                if hit; continue; end
                
                adj(vertexA_idx, vertexB_idx) = 1;
                wAdj(vertexA_idx, vertexB_idx) = norm(xy(:,vertexA_idx) - xy(:,vertexB_idx));
                
                if options.debug
                    plot(axs,[xy(1,vertexA_idx), xy(1,vertexB_idx)],[xy(2,vertexA_idx), xy(2,vertexB_idx)], 'm', 'Tag', 'DEBUG', 'Linewidth', .5);
                    drawnow;
                end
            end
        end

        % replace upper triangle to make adjacency matrices symmetric
        wAdj = fillmissing((wAdj-tril(wAdj)), 'constant', 0) + tril(wAdj).';
        adj = (adj-triu(adj))+tril(adj).';
        
    end

end