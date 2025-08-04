function touch = checkAdjacent(rectA, rectB)

    touch = false;
    for a=1:4
        nexta = a+1;
        if a == 4
            nexta = 1;
        end
        Ca = fitSegment([rectA.vertices(:, a)], [rectA.vertices(:, nexta)]);
        for b=1:4
            nextb = b+1;
                if b == 4
                    nextb = 1;
                end
            Cb = fitSegment([rectB.vertices(:, b)], [rectB.vertices(:, nextb)]);
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