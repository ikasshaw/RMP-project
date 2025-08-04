% Isaac Shaw
% Robot Motion Planning
% 8/06/2025

function [adj, wAdj, xyq] = approxCellGraph3d(A, q_init, q_goal, B, bounds, options)

    arguments
        A (2, :) double
        q_init (3,1) double
        q_goal (3,1) double
        B cell
        bounds (2, :) double
        options.debug logical = true
        options.debugPlotAllCells (1,1) double = false
        options.debugCheckAdjLogic (1,1) double = false
        options.debugPlotEmpties (1,1) double = false
        options.debugPlotEmptiesAdjacencyCheck = false
        options.maxDepth double = 10
        options.minRectScale double = 30
        options.brute logical = false
        options.dtheta (1,1) double = .3
        options.considerBounds logical = false
        options.vizlayers (1,1) {mustBeInteger} = 3
    end

    % Input checks (unchanged)
    checkQ(q_init);
    checkQ(q_goal);
    checkVertices(A);
    checkVertices(bounds);

    if ~inpolygon(q_init(1), q_init(2), bounds(1,:), bounds(2,:))
        error('All points must be inside the boundary')
    elseif ~inpolygon(q_goal(1), q_goal(2), bounds(1,:), bounds(2,:))
        error('All points must be inside the boundary')
    end

    for i=1:numel(B)
        checkVertices(B{i});
        if ~inpolygon(B{i}(1,:), B{i}(2,:), bounds(1,:), bounds(2,:))
            error('All points must be inside the boundary')
        end
    end

    % General setup
    thetas = 0:options.dtheta:2*pi;
    thetas(end+1:end+2) = [q_init(3), q_goal(3)];
    thetas = sort(thetas);

    L = numel(thetas);
    layers_CBs = cell(1,L);
    layers_polyshape_union = cell(1,L);

    for i = 1:L
        theta = thetas(i);
        CBs = cellfun(@(b) cObstacle(theta, A, b), B, 'UniformOutput', false);
        ps_cells = cellfun(@(cb) polyshape(cb.', 'Simplify', false), CBs, 'UniformOutput', false);
        layer_ps = polyshape;
        
        for k = 1:numel(ps_cells)
            layer_ps = union(layer_ps, ps_cells{k});
        end
        
        layers_CBs{i} = CBs;
        layers_polyshape_union{i} = layer_ps;
    end

    boundsPolyshape = polyshape(bounds.', 'Simplify', false);
    verts = [q_init(1:2,:), q_goal(1:2,:), bounds];
    vertices_x_lims = [min(verts(1,:)), max(verts(1,:))];
    vertices_y_lims = [min(verts(2,:)), max(verts(2,:))];

    initialxyBounds = [vertices_x_lims(1), vertices_x_lims(2), vertices_x_lims(2), vertices_x_lims(1);...
        vertices_y_lims(1), vertices_y_lims(1), vertices_y_lims(2), vertices_y_lims(2)];

    if options.debug
        % Plot environment and Cobstacles in debug only
        if isempty(groot().Children)
            fig = figure('Name', 'approxCellGraph - DEBUG');
        else
            fig = gcf;
        end
        hold on
        axs = gca;
        set(axs, 'Parent', fig, 'NextPlot', 'add', 'DataAspectRatio', [1,1,1])
        
        for obs_idx=1:numel(B)
            for l=1:options.vizlayers:L
                CB = layers_CBs{l}{obs_idx};
                CB(3,:) = thetas(l);
                fill3(CB(1,:), CB(2,:), CB(3,:), 'g')
            end
        end

        plot3(axs, q_init(1), q_init(2), q_init(3), 'dg', 'MarkerSize', 30, 'LineWidth', 3);
        plot3(axs, q_goal(1), q_goal(2), q_goal(3), 'dr', 'MarkerSize', 30, 'LineWidth', 3);

        patch('Vertices', bounds.', 'Faces', 1:size(bounds, 2), 'Parent', axs, ...
            'EdgeColor', 'k', 'FaceColor', 'None', 'LineWidth', 1.5);

        [finit, ~] = plotRobot(q_init, A, 'world', true);
        fInittrans = hgtransform('Matrix', Tz(q_init(3)));

        [fgoal, ~] = plotRobot(q_goal, A, 'world', true);
        fGoaltrans = hgtransform('Matrix', Tz(q_goal(3)));

        set(finit, 'Parent', fInittrans);
        set(fgoal, 'Parent', fGoaltrans);

        drawnow;

    end

    % Setup root cell
    C = struct( ...
        'xybounds', initialxyBounds, ...
        'layer_idxs', uint32(1:L), ...
        'subdivide', false, ...
        'parentID', uint32(0), ...
        'id', uint32(1), ...
        'contains_qinit', true, ...
        'contains_qgoal', true, ...
        'cellObstacleIntersection', CellClass.Mixed, ...
        'depth', 0, ...
        'layerClassifications', CellClass.empty(1,0), ...
        'layerCellPolyshape', polyshape(initialxyBounds.', 'Simplify', false), ...
        'diag', norm(diff(initialxyBounds(:, [1,3]), 1,2)), ...
        'centroid', zeros(1,2), ...
        'isLeaf', true, ...
        'children', zeros(1,8,'uint32'), ...
        'indexInParent', 0,...
        'LEFT', zeros(1,'uint32'), ...
        'RIGHT', zeros(1,'uint32'), ...
        'FRONT', zeros(1,'uint32'), ...
        'BACK', zeros(1,'uint32'), ...
        'TOP', zeros(1,'uint32'), ...
        'BOTTOM', zeros(1,'uint32'), ...
        'cx', 0, 'cy', 0, 'theta_mean', 0, ...
        'centroidOfCellInBounds', false, ...
        'cellBoundaryType', CellClass.Mixed, ...
        'cobs', [] ...
        );

    [cx, cy] = centroid(C.layerCellPolyshape);
    C.centroid = [cx, cy];
    C.cx = cx; C.cy = cy; C.theta_mean = mean(thetas(C.layer_idxs));

    PROTOTYPE_CELL = C;

    % Cell storage
    all_cells = repmat(C, 1);
    all_cells(1) = C;
    nodeCount = uint32(1);

    % Top down view (ie Top is out of screen, bottom is into screen) of prism
    %       Front
    % Left         Right
    %       Back

    % The lists are ordered such that when two split prisms are set next to eachother,
    % the indices for the children cells on one prisms face line up with the child cell on the other prisms face

    % Looking at prism from "right through to left"
    right_children = [2, 3, 7, 6];
    left_children = [1, 4, 8, 5];

    % Looking at prism from "back through to front"
    back_children = [1, 2, 6, 5];
    front_children = [4, 3, 7, 8];

    % Looking at prism from "Top through to bottom"
    top_children = [5, 6, 7, 8];
    bottom_children = [1, 2, 3, 4];

    faces = ["LEFT", "RIGHT", "FRONT", "BACK", "TOP", "BOTTOM"];
    opposite_faces = ["RIGHT", "LEFT", "BACK", "FRONT", "BOTTOM", "TOP"];
    face_cell_lists = {left_children, right_children, front_children, back_children, top_children, bottom_children};
    opposite_face_cell_lists = {right_children, left_children, back_children, front_children, bottom_children, top_children};

    stack = uint32(1);         % IDs only
    empty_ids = uint32([]);    % store empty leaf IDs
    full_ids  = uint32([]);

    minRectDiag = C.diag / options.minRectScale;

    q_init_cell = uint32(1);
    q_goal_cell = uint32(1);
    q_init_in_empty_cell = false;
    q_goal_in_empty_cell = false;

    while ~isempty(stack)
        
        curID = stack(1); stack(1) = [];
        curCell = all_cells(curID);
        
        % Optional bounds consideration
        if options.considerBounds

            % Check that the cell is inside the bounds to start with. If fully outside, don't waste time processing further
            if curCell.cellBoundaryType == CellClass.Empty
                all_cells(curID) = curCell; % Update the cell but move on as it is out of the boundary
                continue
            end
        end
        
        % Check each layer for cell intersections
        curCell.layerClassifications = CellClass.empty(1,0);
        for i=1:numel(curCell.layer_idxs)
            layer_idx = curCell.layer_idxs(i);
            layerObsUnion = layers_polyshape_union{layer_idx};
            curCell.layerClassifications(end+1) = classifyIntersection(curCell.layerCellPolyshape, layerObsUnion);
        end
        
        if any(curCell.layerClassifications == CellClass.Mixed)
            curCell.subdivide = true;
            curCell.cellObstacleIntersection = CellClass.Mixed;
            
        elseif all(curCell.layerClassifications == CellClass.Empty)
            curCell.cellObstacleIntersection = CellClass.Empty;
            if curCell.contains_qinit
                q_init_in_empty_cell = true;
                q_init_cell = curCell.id;
            end
            if curCell.contains_qgoal
                q_goal_in_empty_cell = true;
                q_goal_cell = curCell.id;
            end
            empty_ids(end+1) = curCell.id;
            a = plot(curCell.centroid(1), curCell.centroid(2), 'r*');
            b = plot(curCell.layerCellPolyshape);
            delete(a)
            delete(b)
            
        elseif all(curCell.layerClassifications == CellClass.Full)
            curCell.cellObstacleIntersection = CellClass.Full;
            full_ids(end+1) = curCell.id;
            all_cells(curCell.id) = curCell;
            continue
        end
        
        % Optional boundary consideration
        if options.considerBounds
            % Ensure no empty cells are considered valid if they are outside the workspace
            if curCell.cellObstacleIntersection == CellClass.Empty
                if ~curCell.centroidOfCellInBounds

                    if ~isempty(empty_ids) && empty_ids(end)==curID
                        empty_ids(end) = []; % remove last append if we just added it
                    end
                    curCell.subdivide = true;

                elseif curCell.cellBoundaryType == CellClass.Empty
                    if ~isempty(empty_ids) && empty_ids(end)==curID
                        empty_ids(end) = [];
                    end
                    all_cells(curID) = curCell;
                    continue
                end
            end
        end

        % Split Cells
        if curCell.subdivide
            % curCell
            % pause
            curCell.isLeaf = false;
            
            if options.debug
                plt = findall(gca, 'Tag', string(curCell.parentID)); % delete current viz, not parent
                if ~isempty(plt), delete(plt); end
            end
            
            % stop splitting if too deep/small — prune stack to only needed branches
            if curCell.depth >= options.maxDepth || minRectDiag > curCell.diag/2
                if q_init_in_empty_cell && q_goal_in_empty_cell
                    break
                else
                    newStack = uint32(1);         % IDs only
                    foundqinit = false; foundqgoal = false;
                    % Only want to make sure the start and end positions are in empty cells
                    while ~isempty(stack)
                        c = stack(1); stack(1) = [];
                        c = all_cells(c);
                        if c.contains_qinit && c.isLeaf, foundqinit = true; newStack(end+1) = c.id; end
                        if c.contains_qgoal && c.isLeaf, foundqgoal = true; newStack(end+1) = c.id; end
                        if foundqinit && foundqgoal, break; end
                    end
                    stack = newStack;
                end
            end
            
            [children, nodeCount] = splitCell(curCell, nodeCount, q_init, q_goal, thetas, PROTOTYPE_CELL, boundsPolyshape, options.considerBounds);
            
            for k=1:numel(children)
                switch k
                case 1
                    children{k}.RIGHT(1) = children{2}.id;
                    children{k}.FRONT(1) = children{4}.id;
                    children{k}.TOP(1) = children{5}.id;
                case 2
                    children{k}.LEFT(1) = children{1}.id;
                    children{k}.FRONT(1) = children{3}.id;
                    children{k}.TOP(1) = children{6}.id;
                case 3
                    children{k}.LEFT(1) = children{4}.id;
                    children{k}.BACK(1) = children{2}.id;
                    children{k}.TOP(1) = children{7}.id;
                case 4
                    children{k}.RIGHT(1) = children{3}.id;
                    children{k}.BACK(1) = children{1}.id;
                    children{k}.TOP(1) = children{8}.id;
                case 5
                    children{k}.RIGHT(1) = children{6}.id;
                    children{k}.FRONT(1) = children{8}.id;
                    children{k}.BOTTOM(1) = children{1}.id;
                case 6
                    children{k}.LEFT(1) = children{5}.id;
                    children{k}.FRONT(1) = children{7}.id;
                    children{k}.BOTTOM(1) = children{2}.id;
                case 7
                    children{k}.LEFT(1) = children{8}.id;
                    children{k}.BACK(1) = children{6}.id;
                    children{k}.BOTTOM(1) = children{3}.id;
                case 8
                    children{k}.RIGHT(1) = children{7}.id;
                    children{k}.BACK(1) = children{5}.id;
                    children{k}.BOTTOM(1) = children{4}.id;
                end
            end
            
            %% Modify child and neighbor adjacencies
            for face_idx = 1:numel(faces)
                
                face = faces(face_idx);
                faces_children = face_cell_lists{face_idx};
                opposite_face = opposite_faces(face_idx);
                opposite_faces_children = opposite_face_cell_lists{face_idx};
                
                %% Modify child and neighbor adjacencies
                for i =1:numel(curCell.(face))
                    id = curCell.(face)(i);
                    if id == 0, continue; end
                    neighbor = all_cells(id);
                    neighbor.(opposite_face) = neighbor.(opposite_face)(neighbor.(opposite_face) ~= curCell.id);
                    
                    if neighbor.depth < children{1}.depth
                        for child_id = faces_children
                            neighbor.(opposite_face)(end+1) = children{child_id}.id;
                            children{child_id}.(face)(end+1) = neighbor.id;
                        end
                    elseif neighbor.depth == children{1}.depth
                        for idx=1:4
                            if neighbor.indexInParent == opposite_faces_children(idx)
                                neighbor.(opposite_face)(end+1) = children{faces_children(idx)}.id;
                                children{faces_children(idx)}.(face)(end+1) = neighbor.id;
                            end
                        end
                    end

                    all_cells(id) = neighbor;

                end
            end
            
            % store child IDs on parent and in global array
            childIDs = zeros(1,8,'uint32');
            
            for k=1:8
                childIDs(k) = children{k}.id;
                stack(end+1) = children{k}.id;
                all_cells(children{k}.id) = children{k};
            end
            curCell.children = childIDs;
            
            all_cells(curID) = curCell;
            
        else
            all_cells(curCell.id) = curCell;
        end

        if options.debug && (mod(curCell.id, 200)==0), drawnow limitrate; end
    end

    n = numel(empty_ids) + 2;
    adj  = false(n, n);
    wAdj = inf(n, n);

    xyq = zeros(3, n);
    xyq(:,1) = q_init;
    xyq(:,end) = q_goal;

    % Get all empty cell positions
    for i=1:numel(empty_ids)
        C = all_cells(empty_ids(i));
        xyq(:, i+1) = [C.cx; C.cy; C.theta_mean];
    end

    for idxA = 1:n-2
        a_cell_id = empty_ids(idxA);
        A = all_cells(a_cell_id);

        neighbors = [A.LEFT, A.RIGHT, A.FRONT, A.BACK, A.TOP, A.BOTTOM];

        % Add a connection from the initial position to the containing empty cell
        if q_init_cell == a_cell_id
            adj(1,idxA+1) = 1;
            wAdj(1, idxA+1) = norm(xyq(:,1) - xyq(:,idxA));

            adj(idxA+1, 1) = 1;
            wAdj(idxA+1, 1) = norm(xyq(:,1) - xyq(:,idxA));
        end
        
        % Add a connection from the goal position to the containing empty cell
        if q_goal_cell == A.id
            adj(n, idxA+1) = 1;
            wAdj(n, idxA+1) = norm(xyq(:,n) - xyq(:,idxA));

            adj(idxA+1, n) = 1;
            wAdj(idxA+1, n) = norm(xyq(:,n) - xyq(:,idxA));
        end
        
        % Loop through the rest of the empty cells and add a connection if they are in the current cells neighbors
        for idxB = [idxA:n-2]

            b_cell_id = empty_ids(idxB);
            B = all_cells(b_cell_id);

            if ismember(b_cell_id, neighbors) && B.cellObstacleIntersection == CellClass.Empty

                adj(idxA+1, idxB+1) = 1;
                wAdj(idxA+1, idxB+1) = norm(xyq(:,idxA) - xyq(:,idxB));
                
                adj(idxB + 1, idxA+1) = 1;
                wAdj(idxB+1, idxA+1) = norm(xyq(:,idxA) - xyq(:,idxB));
            end
        end
    end

    if options.debug
        
        disp('Debug Plotting Beginning')
        % ax = gca; hold(ax,'on');
        if options.debugPlotEmpties
            for cellid = empty_ids

                curDebugCell = all_cells(cellid);

                if ~curDebugCell.isLeaf
                    continue
                end

                up = curDebugCell.xybounds; lo = circshift(curDebugCell.xybounds, 1, 2);
                up(end+1, :) = thetas(curDebugCell.layer_idxs(1));
                lo(end+1, :) = thetas(curDebugCell.layer_idxs(end));
                pts = [up, lo];
                
                cellAlphaShape = alphaShape(pts.');
                
                plot(cellAlphaShape, 'FaceAlpha', .1, 'FaceColor', 'g', 'EdgeAlpha', .1, 'EdgeColor', 'g', 'Tag', string(curDebugCell.id));

                if mod(cellid, 30) == 0
                    drawnow;
                end
            end
        end 

        if options.debugPlotAllCells
            for cellid = 1:numel(all_cells)

                curDebugCell = all_cells(cellid);

                if ~curDebugCell.isLeaf
                    continue
                end

                up = curDebugCell.xybounds; lo = circshift(curDebugCell.xybounds, 1, 2);
                up(end+1, :) = thetas(curDebugCell.layer_idxs(1));
                lo(end+1, :) = thetas(curDebugCell.layer_idxs(end));
                pts = [up, lo];
                
                cellAlphaShape = alphaShape(pts.');
                
                if curDebugCell.cellBoundaryType == CellClass.Empty
                    plot3(curDebugCell.centroid(1), curDebugCell.centroid(2), curDebugCell.theta_mean, 'Or', 'Tag', string(curDebugCell.id), 'MarkerSize', 1/curDebugCell.depth * 20);
                    % plot(cellAlphaShape, 'FaceAlpha', .1, 'FaceColor', 'r', 'EdgeColor', 'none', 'Tag', string(curDebugCell.id));
                elseif ~curDebugCell.centroidOfCellInBounds
                    % plot(cellAlphaShape, 'FaceAlpha', .1, 'FaceColor', 'y', 'EdgeColor', 'none', 'Tag', string(curDebugCell.id));
                    plot3(curDebugCell.centroid(1), curDebugCell.centroid(2), curDebugCell.theta_mean, 'Sy', 'Tag', string(curDebugCell.id), 'MarkerSize', 1/curDebugCell.depth * 20);
                elseif curDebugCell.cellObstacleIntersection == CellClass.Mixed
                    plot3(curDebugCell.centroid(1), curDebugCell.centroid(2), curDebugCell.theta_mean, 'Sm', 'Tag', string(curDebugCell.id), 'MarkerSize', 1/curDebugCell.depth * 20);
                    % plot(cellAlphaShape, 'FaceColor', 'none', 'EdgeColor', 'm', 'EdgeAlpha', 1.0, 'Tag', string(curDebugCell.id));
                elseif curDebugCell.cellObstacleIntersection == CellClass.Empty
                    plot(cellAlphaShape, 'FaceAlpha', .1, 'FaceColor', 'g', 'EdgeAlpha', .1, 'EdgeColor', 'g', 'Tag', string(curDebugCell.id));
                elseif curDebugCell.cellObstacleIntersection == CellClass.Full
                    plot3(curDebugCell.centroid(1), curDebugCell.centroid(2), curDebugCell.theta_mean, 'Sr', 'Tag', string(curDebugCell.id), 'MarkerSize', 1/curDebugCell.depth * 20);
                    % plot(cellAlphaShape, 'FaceAlpha', .1, 'FaceColor', 'r', 'EdgeColor', 'none', 'Tag', string(curDebugCell.id));
                end

                if mod(cellid, 30) == 0
                    drawnow;
                end
            end
        end 

        if options.debugPlotEmptiesAdjacencyCheck

            for i = 1:numel(empty_ids)
                cellid = empty_ids(i);

                curDebugCell = all_cells(cellid);
                
                ps = [];
                
                up = curDebugCell.xybounds; lo = circshift(curDebugCell.xybounds, 1, 2);
                up(end+1, :) = thetas(curDebugCell.layer_idxs(1));
                lo(end+1, :) = thetas(curDebugCell.layer_idxs(end));
                pts = [up, lo];
                
                cellAlphaShape = alphaShape(pts.');
                
                % plot(cellAlphaShape, 'FaceAlpha', .5, 'FaceColor', 'g', 'EdgeAlpha', .1, 'EdgeColor', 'g');
                ps(end+1) = plot(cellAlphaShape, 'FaceAlpha', .5, 'FaceColor', 'g', 'EdgeAlpha', .1, 'EdgeColor', 'g');

                neighbors = [curDebugCell.LEFT, curDebugCell.RIGHT, curDebugCell.FRONT, curDebugCell.BACK, curDebugCell.TOP, curDebugCell.BOTTOM];
                
                for k = 1:numel(neighbors)
                    j = neighbors(k);
                    
                    if ismember(j, curDebugCell.LEFT)
                        color = 'y';
                    elseif ismember(j, curDebugCell.RIGHT)
                        color = 'r';
                    elseif ismember(j, curDebugCell.FRONT)
                        color = 'c';
                    elseif ismember(j, curDebugCell.BACK)
                        color = 'm';
                    elseif ismember(j, curDebugCell.TOP)
                        color = 'k';
                    elseif ismember(j, curDebugCell.BOTTOM)
                        color = 'b';
                    end
                    
                    if j == 0
                        continue;
                    end
                    c = all_cells(j);
                    
                    up = c.xybounds; lo = circshift(c.xybounds, 1, 2);
                    up(end+1, :) = thetas(c.layer_idxs(1));
                    lo(end+1, :) = thetas(c.layer_idxs(end));
                    pts = [up, lo];
                    cellAlphaShape = alphaShape(pts.');
                    
                    ps(end+1) = plot(cellAlphaShape, 'FaceAlpha', .1, 'FaceColor', color, 'EdgeAlpha', .1, 'EdgeColor', color);
                    
                end
                drawnow;
                disp("Press any key to move to next cell")
                pause()

                while ~isempty(ps)
                    delete(ps(1));
                    ps(1) = [];
                end
            end
        end
 
        if options.debugCheckAdjLogic

            for cellid = 1:numel(all_cells)
                ps = [];
                
                up = curDebugCell.xybounds; lo = circshift(curDebugCell.xybounds, 1, 2);
                up(end+1, :) = thetas(curDebugCell.layer_idxs(1));
                lo(end+1, :) = thetas(curDebugCell.layer_idxs(end));
                pts = [up, lo];
                
                cellAlphaShape = alphaShape(pts.');
                
                ps(end+1) = plot(cellAlphaShape, 'FaceAlpha', .5, 'FaceColor', 'g', 'EdgeAlpha', .1, 'EdgeColor', 'g');

                neighbors = [curDebugCell.LEFT, curDebugCell.RIGHT, curDebugCell.FRONT, curDebugCell.BACK, curDebugCell.TOP, curDebugCell.BOTTOM];
                
                for k = 1:numel(neighbors)
                    j = neighbors(k);
                    
                    if ismember(j, curDebugCell.LEFT)
                        color = 'y';
                    elseif ismember(j, curDebugCell.RIGHT)
                        color = 'r';
                    elseif ismember(j, curDebugCell.FRONT)
                        color = 'c';
                    elseif ismember(j, curDebugCell.BACK)
                        color = 'm';
                    elseif ismember(j, curDebugCell.TOP)
                        color = 'k';
                    elseif ismember(j, curDebugCell.BOTTOM)
                        color = 'b';
                    end
                    
                    if j == 0
                        continue;
                    end
                    c = all_cells(j);
                    
                    up = c.xybounds; lo = circshift(c.xybounds, 1, 2);
                    up(end+1, :) = thetas(c.layer_idxs(1));
                    lo(end+1, :) = thetas(c.layer_idxs(end));
                    pts = [up, lo];
                    cellAlphaShape = alphaShape(pts.');
                    
                    ps(end+1) = plot(cellAlphaShape, 'FaceAlpha', .1, 'FaceColor', color, 'EdgeAlpha', .1, 'EdgeColor', color);
                    
                end
                drawnow;
                disp("Press any key to move to next cell")
                pause()

                while ~isempty(ps)
                    delete(ps(1));
                    ps(1) = [];
                end
            end
        end
    end
end







