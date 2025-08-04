% Isaac Shaw
% Robot Motion Planning
% 7/13/2025

function [adj, wAdj, xy] = vCellGraph(q_init, q_goal, CB, bounds, options)

    arguments
        q_init double
        q_goal double
        CB cell
        bounds double
        options.debug = false
        options.boundaryEdges = false
    end

    % Input variable checks
    checkQ(q_init);
    checkQ(q_goal);
    checkVertices(bounds);
    % Check that all points are within the bounds
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

    debug = options.debug;

    if debug
        axs = gca;
        hold(axs, 'on');
        daspect(axs,[1,1,1]);
        % kids = findobj(axs, 'Tag', 'DEBUG');
        % delete(kids);
        currentSegment = plot(axs, nan, nan, 'r', 'Tag', 'Current Segment', 'LineWidth', 1);
        currentPoint = plot(axs, nan, nan, '*r', 'Tag', 'Current Vertex');
    end

    % For each obstacle, add the vertices to the xy array, note the
    % obstacle num, and compute all obstacle edge line segments
    Obs_segments = cell(numel(CB)*2);
    xyInit(:,1) = q_init(1:2,:);
    cbIDX(1,1) = 0; % Bookkeeping array to map back to corresponding CB
    cbVertexIdx(1,1) = 0; % Bookkeeping array to map vertex number within obstacles
    for idx = 1:numel(CB)
        cb = CB{idx};
        cbN = size(CB{idx}, 2);
        cbidx_segments = zeros(2, cbN*2);
        for vertex = 1:size(cb, 2)
            xyInit(:, end+1) = cb(:,vertex);
            cbIDX(:,end+1) = idx;
            cbVertexIdx(:,end+1) = vertex;

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
    xyInit(:,end+1) = q_goal(1:2,:);
    cbIDX(:, end+1) = 0;
    cbVertexIdx(:, end+1) = 0;

    Xlims = [min(bounds(1,:)), max(bounds(1,:))];
    Ylims = [min(bounds(2,:)), max(bounds(2,:))];

    if options.boundaryEdges
        xyInit(:,end+1:end+2) = [Xlims; Ylims]; % Uncomment to include boundaries
        cbIDX(:,end+1:end+2) = 0; % Uncomment to include boundaries
        % cbIDX(:,end+1:end+2) = 0; % Uncomment to include boundaries
    end
    
    M = size(xyInit, 2)-1;

    % Initial segments for vertical lines at each vertex
    for i=2:M
        bottomVertex = [xyInit(1,i); Ylims(1)];
        topVertex =[xyInit(1,i); Ylims(2)];
        allCs{i} = fitSegment(bottomVertex, topVertex);

        if debug
            set(currentSegment, 'XData', [bottomVertex(1), topVertex(1)], 'YData', [bottomVertex(2), topVertex(2)]);
            set(currentPoint, 'XData', xyInit(1,i), 'YData', xyInit(2,i));
            drawnow;
        end
    end

    % Draw all vertical lines
    P = size(bounds, 2);
    for i=2:M
        Co = allCs{i};
        intPoints{i} = [];

        if debug
            seg = Co*[0,1; 1, 1];
            set(currentSegment, 'XData', seg(1,:), 'YData', seg(2,:));
            drawnow;
        end

        for j = 1:P
            nextJ = j + 1;
            if j == P
                nextJ = 1;
            end
            v1 = bounds(:,j);
            v2 = bounds(:,nextJ);
            Cs = fitSegment(v1, v2);

            [intersect, internalIntersect, intersectionPoints] = intersectSegment(Co, Cs);

            if intersect
                intPoints{i}(:,end+1) = intersectionPoints(:,1);
            end
        end

        tfMinY = intPoints{i}(2,:) == min(intPoints{i}(2,:));
        tfMaxY = intPoints{i}(2,:) == max(intPoints{i}(2,:));
        v1 = intPoints{i}(:,tfMinY);
        v2 = intPoints{i}(:,tfMaxY);
        v1 = v1(:,1);
        v2 = v2(:,1);
        allCs{i} = fitSegment(v1, v2);

        if debug
            plt_seg(i) = plot(axs, [v1(1), v2(1)], [v1(2), v2(2)], 'r');
            drawnow;
        end
    end

    % Cut lines
    % Subtract 1 if including the left and right limits 
    for i=2:M

        Co = allCs{i};
        intPoints{i} = [];
        seg = Co*[0,1; 1, 1];

        if debug
            seg = Co*[0,1; 1, 1];
            set(currentSegment, 'XData', seg(1,:), 'YData', seg(2,:));
            drawnow;
        end

        for j = 1:numel(CB)

            tfOwnObstacle = false;

            K = size(CB{j}, 2);

            if cbIDX(i) == j
                tfOwnObstacle = true;
            end

            for k =1:K

                nextK = k+1;
                if k == K
                    nextK = 1;
                end

                v1 = CB{j}(:,k);
                v2 = CB{j}(:,nextK);
                Cs = fitSegment(v1, v2);

                [intersect, internalIntersect, intersectionPoints] = intersectSegment(Co, Cs);

                if intersect
                    if ~tfOwnObstacle
                        if xyInit(2,i) > intersectionPoints(2)
                            v1 = intersectionPoints;
                            v2 = seg(:,2);
                            allCs{i} = fitSegment(v1, v2);
                        elseif xyInit(2,i) < intersectionPoints(2)
                            v1 = seg(:,1);
                            v2 = intersectionPoints;
                            allCs{i} = fitSegment(v1, v2);
                        else
                            disp(seg);
                            disp(intersectionPoints);
                            error('Unexpected Case')
                        end

                    else
                        if k == cbVertexIdx(i) || nextK == cbVertexIdx(i)
                            continue
                        end

                        if intersect
                            if xyInit(2,i) > intersectionPoints(2)
                                v1 = xyInit(:,i);
                                v2 = seg(:,2);
                                allCs{i} = fitSegment(v1, v2);
                            elseif xyInit(2,i) < intersectionPoints(2)
                                v1 = seg(:,1);
                                v2 = xyInit(:,i);
                                allCs{i} = fitSegment(v1, v2);
                            else
                                disp(seg);
                                disp(intersectionPoints);
                                error('Unexpected Case')
                            end
                        end
                    end

                    allCs{i} = fitSegment(v1, v2);
                    Co = allCs{i};
                    seg = Co*[0,1; 1, 1];
                    if debug
                        set(plt_seg(i), 'XData', seg(1,:), 'YData', seg(2,:))
                        drawnow;
                    end
                end
            end
        end
    end

    ZERO = 1e-8;
    xy(:,1) = q_init(1:2,:);

    for i =2:M
        Co = allCs{i};
        xyTMP = Co*[0, 1; 1, 1];
        d(1) = norm( xyTMP(:,1) - xyInit(:,i));
        d(2) = norm( xyTMP(:,2) - xyInit(:,i));
        if any(d < ZERO)
            xy(:,end+1) = Co * [0.5; 1];
        else
            xy(:,end+1) = mean([xyTMP(:,1), xyInit(:,i)], 2);
            xy(:,end+1) = mean([xyTMP(:,2), xyInit(:,i)], 2);
        end
    end
    xy(:, end+1) = q_goal(1:2,1);

    [B, order] = sortrows(xy.', [1,2]);
    xy = B.';

    if debug
        plt_center = plot(axs, xy(1,:), xy(2,:), 'xb');
        drawnow;
    end

    n = size(xy, 2);
    adj = zeros(n, n);
    wAdj = inf(n, n);

    % Main loops to check for valid segments between each pair of vertices
    % Undirected adjacency matrices are symmetric so only iterate over
    % lower triangle
    for vertexA_idx=1:n
        for vertexB_idx =1:vertexA_idx
            if vertexA_idx == vertexB_idx
                wAdj(vertexA_idx, vertexB_idx) = 0;
                continue
            end

            if debug
                set(currentSegment, 'XData', [xy(1,vertexA_idx), xy(1,vertexB_idx)], 'YData', [xy(2,vertexA_idx), xy(2,vertexB_idx)]);
                drawnow;
            end

            Cs = fitSegment(xy(:, vertexA_idx), xy(:, vertexB_idx));
            hit = false;

            for idx = 1:numel(CB)

                cb = CB{idx};

                [~, cbN] = size(cb);

                cur_obs_segments = Obs_segments{idx};

                for edgeIdx=1:cbN

                    Co = cur_obs_segments(:, edgeIdx*2-1:edgeIdx*2);

                    [intersects, internallyIntersects, ~] = intersectSegment(Co, Cs);

                    if intersects; hit = true; break; end

                end
                if hit; break; end
            end

            if hit; continue; end

            adj(vertexA_idx, vertexB_idx) = 1;
            wAdj(vertexA_idx, vertexB_idx) = norm(xy(:,vertexA_idx) - xy(:,vertexB_idx));

            if debug
                plot(axs,[xy(1,vertexA_idx), xy(1,vertexB_idx)],[xy(2,vertexA_idx), xy(2,vertexB_idx)], 'm', 'Tag', 'DEBUG', 'Linewidth', .5);
                drawnow;
            end
        end
    end

    if debug
        set(currentSegment, 'Visible', 'off');
        set(currentPoint, 'Visible', 'off');
    end

    % replace upper triangle to make adjacency matrices symmetric
    wAdj = fillmissing((wAdj-triu(wAdj)), 'constant', 0) + tril(wAdj).';
    adj = (adj-triu(adj))+tril(adj).';
 
end