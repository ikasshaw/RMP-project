function touch = checkAdjacent3d(cellA, cellB)

    touch = false;

    sameLayer = ismember(cellA.layer_idxs, cellB.layer_idxs);
    % Adjacent cells must share at least one layer
    if any(sameLayer)
        
        % Top/bottom face interesection
        intersection = intersect(cellA.layerCellPolyshape, cellB.layerCellPolyshape);

        if intersection.NumRegions > 0
            touch = true;
            return
        end
        
        % Check sides adjacencies. 
        % Truly adjacent if both have more than 1 layer and more than 2 layers are adjacent, or if one of the cells has a single layer and it is in the other cells layers 
        if numel(cellA.layer_idxs) == 1 || numel(cellB.layer_idxs) == 1 || sum(sameLayer, 'all') > 2
            for a=1:4
                nexta = a+1;
                if a == 4
                    nexta = 1;
                end
                Ca = fitSegment([cellA.xybounds(:, a)], [cellA.xybounds(:, nexta)]);
                for b=1:4
                    nextb = b+1;
                        if b == 4
                            nextb = 1;
                        end
                    Cb = fitSegment([cellB.xybounds(:, b)], [cellB.xybounds(:, nextb)]);
                    [intersects, ~ , points] = intersectSegment(Ca, Cb);
                    % Only a true intersection if more than 1 point intersect
                    if intersects & size(points, 2) > 1
                        touch = true;
                        break
                    end
                end
                if touch
                    break
                end
            end
        end
    end
end