% Isaac Shaw
% Robot Motion Planning
% 7/26/2025

function state = classifyIntersection(A, B, options)
    % R: polyshape of rectangle
    % area of rectangle (polyarea is fast for simple polygons)
    arguments
        A polyshape
        B polyshape
        options.boundsCheck logical = false
    end

    % if A.area == 0
    %     state = CellClass.Empty;
    %     return
    % end

    state = CellClass.Empty;

    % if ~options.boundsCheck

        % if ~overlaps(A, B)
        %     return
        % end

        inter = intersect(A, B);
        subtracted = subtract(A, inter);

        if subtracted.NumRegions == 0
            state = CellClass.Full;
        elseif inter.NumRegions > 0
            state = CellClass.Mixed;
        end

    % else

    %     interCellBoundary = intersect(A, B);
        
    %     if interCellBoundary.NumRegions == 0
    %         state = CellClass.Full;
    %     else
    %         subtractedCellBoundary = subtract(A, interCellBoundary);
    %         if subtractedCellBoundary.NumRegions > 0
    %             state = CellClass.Mixed;
    %         end
    %     end

    % end
end
