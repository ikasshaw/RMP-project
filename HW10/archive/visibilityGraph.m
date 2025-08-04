% Isaac Shaw
% Robot Motion Planning
% 7/13/2025

function [adj, wAdj, xy] = visibilityGraph(q_init, q_goal, CB, options)

    arguments
        q_init double
        q_goal double
        CB cell
        options.debug = false
    end

    % Input variable checks
    checkQ(q_init);
    checkQ(q_goal);
    if ~iscell(CB)
        error('CB must be a cell array')
    else
        for i=1:numel(CB)
            checkVertices(CB{i});
        end
    end

    debug = options.debug;

    if debug
        axs = gca;
        hold(axs, 'on');
        daspect(axs,[1,1,1]);
        kids = findobj(axs, 'Tag', 'DEBUG');
        delete(kids);
        currentSegment = plot(axs, nan, nan, 'r', 'Tag', 'DEBUG', 'Linewidth', 1);
    end
    
    % For each obstacle, add the vertices to the xy array, note the
    % obstacle num, and compute all obstacle edge line segments
    Obs_segments = cell(numel(CB)*2);
    xy(:,1) = q_init(1:2,:);
    cbIDX(1,1) = 0; % Bookkeeping array to map back to corresponding CB
    cbVertexIdx(1,1) = 0; % Bookkeeping array to map vertex number within obstacles
    for idx = 1:numel(CB)
        cb = CB{idx};
        cbN = size(CB{idx}, 2);
        cbidx_segments = zeros(2, cbN*2);
        for vertex = 1:size(cb, 2)
            xy(:, end+1) = cb(:,vertex);
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
    xy(:,end+1) = q_goal(1:2,:);
    cbIDX(:, end+1) = 0;
    cbVertexIdx(:, end+1) = 0;

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

            % Inter-obstacle case
            if cbIDX(vertexA_idx) == cbIDX(vertexB_idx) && cbIDX(vertexA_idx) ~= 0

                [~, cbN] = size(CB{cbIDX(vertexA_idx)});
                
                abs_diff = abs(vertexA_idx - vertexB_idx);

                % Adjacent edges diff of 1 except at wrap which are diff of num vertices - 1
                if abs_diff == 1 || abs_diff == (cbN - 1)

                    adj(vertexA_idx, vertexB_idx) = 1;
                    wAdj(vertexA_idx, vertexB_idx) = norm(xy(:,vertexA_idx)-xy(:,vertexB_idx));

                    if debug
                        plot(axs,[xy(1,vertexA_idx), xy(1,vertexB_idx)],[xy(2,vertexA_idx), xy(2,vertexB_idx)], 'm', 'Tag', 'DEBUG', 'Linewidth', 1);
                        drawnow;
                    end
                end
                continue;
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

                    nextEdgeIdx = edgeIdx + 1;

                    if edgeIdx == cbN; nextEdgeIdx = 1; end

                    verts = [cb(:,edgeIdx), cb(:,nextEdgeIdx)];

                    % Only allow vertex intersections if the intersection is one of the current points
                    if idx == cbIDX(vertexA_idx)
                        if ismember(xy(:, vertexA_idx), verts)
                            continue    
                        end
                    end

                    if idx == cbIDX(vertexB_idx)
                        if ismember(xy(:, vertexB_idx), verts)
                            continue    
                        end
                    end

                    if intersects; hit = true; break; end
                    
                end
                if hit; break; end
            end

            if hit; continue; end

            adj(vertexA_idx, vertexB_idx) = 1;
            wAdj(vertexA_idx, vertexB_idx) = norm(xy(:,vertexA_idx)-xy(:,vertexB_idx));
            
            if debug
                plot(axs,[xy(1,vertexA_idx), xy(1,vertexB_idx)],[xy(2,vertexA_idx), xy(2,vertexB_idx)], 'm', 'Tag', 'DEBUG', 'Linewidth', 1);
                drawnow;
            end
        end
    end

    if debug
        set(currentSegment, 'Visible', 'off');
    end

    % replace upper triangle to make adjacency matrices symmetric
    wAdj = fillmissing((wAdj-triu(wAdj)), 'constant', 0) + tril(wAdj).';
    adj = (adj-triu(adj))+tril(adj).';

end
