% function [cells, nodeCount] = splitCell(parentCell, nodeCount, q_init, q_goal, thetas)

function [cells, nodeCount] = splitCell(parentCell, nodeCount, q_init, q_goal, thetas, PROTO, boundsPolyshape, considerBoundary)

    arguments
        parentCell struct
        nodeCount (1,1) {mustBeInteger}
        q_init (3,1) double
        q_goal (3,1) double
        thetas (1, :) double
        PROTO struct
        boundsPolyshape polyshape
        considerBoundary (1,1) logical = false
    end

    cells = cell(1,8);

    subdivided_xy = subDivRec(parentCell.xybounds);

    for child_idx = 1:8
        child = PROTO;                         % << start from prototype
        
        child.id        = uint32(nodeCount + child_idx);
        child.depth     = parentCell.depth + 1;
        child.parentID  = parentCell.id;
        child.subdivide = false;
        child.isLeaf    = true;
        child.children  = zeros(1,8,'uint32');
        child.layerClassifications = CellClass.empty(1,0);
        child.indexInParent = uint8(child_idx);   % so we don’t have to find(...)
        child.cellBoundaryType = parentCell.cellBoundaryType;
        
        child.xybounds = subdivided_xy{mod(child_idx-1,4)+1};

        if child_idx <= 4
            child.layer_idxs = parentCell.layer_idxs(1:ceil(numel(parentCell.layer_idxs)/2));
        else
            n = numel(parentCell.layer_idxs);
            mid = ceil(n/2);
            child.layer_idxs = parentCell.layer_idxs(mid + 1 - mod(n,2) : end);
        end
        
        child.layerCellPolyshape = polyshape(child.xybounds.', 'Simplify', false);
        [cx, cy] = centroid(child.layerCellPolyshape);
        child.centroid  = [cx, cy];
        child.cx        = cx;
        child.cy        = cy;
        child.diag      = norm(diff(child.xybounds(:, [1,3]), 1,2));
        child.contains_qinit = false;
        child.contains_qgoal = false;
        child.theta_mean     = mean(thetas(child.layer_idxs));
        
        % q_init/q_goal flags
        if parentCell.contains_qinit
            if isinterior(child.layerCellPolyshape, q_init(1), q_init(2))
                % tlo = thetas(child.layer_idxs(1)); thi = thetas(child.layer_idxs(end));
                cellThetas = thetas(child.layer_idxs);
                % if (q_init(3) >= tlo && q_init(3) <= thi)
                if ismember(q_init(3), cellThetas)
                    child.contains_qinit = true;
                end
            end
        end

        if parentCell.contains_qgoal
            if isinterior(child.layerCellPolyshape, q_goal(1), q_goal(2))
                % tlo = thetas(child.layer_idxs(1)); thi = thetas(child.layer_idxs(end));
                cellThetas = thetas(child.layer_idxs);
                if ismember(q_goal(3), cellThetas)
                % if (q_goal(3) >= tlo && q_goal(3) <= thi)
                    child.contains_qgoal = true;
                end
            end
        end
        
        child.cellBoundaryType = classifyIntersection(child.layerCellPolyshape, boundsPolyshape);
        child.centroidOfCellInBounds = isinterior(boundsPolyshape, child.centroid(1), child.centroid(2));

        cells{child_idx} = child;
    end
    nodeCount = uint32(nodeCount + 8);

end