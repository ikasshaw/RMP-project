function [intersection, internalIntersect, xy] = intersectSegment(Ci, Cj)

    % Check inputs
    [mi, ni] = size(Ci);
    [mj, nj] = size(Cj);
    
    % Basic checks Ci
    if ~ismatrix(Ci)
        error('Ci must be a 2x2 matrix')
    elseif mi ~= 2 & ni ~= 2
        error('Ci must be 2x2')
    elseif ~isnumeric(Ci) % Check that matrix is all numeric
        error('Ci must be numeric')
    elseif ~isreal(Ci) % Check for all real
        error('Ci must contain all real values')
    end

    % Basic checks Cj
    if ~ismatrix(Cj)
        error('Cj must be a 2x2 matrix')
    elseif mj ~= 2 & nj ~= 2
        error('Cj must be 2x2')
    elseif ~isnumeric(Cj) % Check that matrix is all numeric
        error('Cj must be numeric')
    elseif ~isreal(Cj) % Check for all real
        error('Cj must contain all real values')
    end

    intersection = false;
    internalIntersect = false;
    xy = [];
    ZERO = 1e-10;

    Ai = Ci(:,1);
    Bi = Ci(:,2);

    Aj = Cj(:,1);
    Bj = Cj(:,2);

    Pi = Ci * [0, 1; 1, 1];
    Pj = Cj * [0, 1; 1, 1];

    A = [Ai, -Aj];
    B = Bj - Bi;

    if abs(det(A)) > ZERO
        S = A^(-1) * B;
        si = S(1);
        sj = S(2);

        if (si >= 0 && si <= 1) && (sj >= 0 && sj <= 1)
            intersection = true;
            xy = Ci*[si; 1];
        end

        if (si > 0 && si < 1) && (sj > 0 && sj < 1)
            internalIntersect = true;
        end

    else

        idx = find(abs(Aj) == max(abs(Aj)), 1, 'first');
        sj(1) = (Pi(idx,1) - Bj(idx))/ Aj(idx);
        sj(2) = (Pi(idx,2) - Bj(idx))/ Aj(idx);

        tfIDX = true(1,2);
        tfIDX(idx) = false;

        tf_sj = sj >= 0 & sj <= 1;
        if any(tf_sj)
            tf_Pi = Pi(tfIDX, tf_sj) == Aj(tfIDX)*sj(tf_sj) + Bj(tfIDX);
            if any(tf_Pi)
                intersection = true;
                tmp_sj = sj(tf_sj);
                xy = Cj*[tmp_sj(tf_Pi); ones(1, nnz(tf_Pi))];
            end
        end

        idx = find(abs(Ai) == max(abs(Ai)), 1, 'first');
        si(1) = (Pj(idx,1) - Bi(idx))/ Ai(idx);
        si(2) = (Pj(idx,2) - Bi(idx))/ Ai(idx);

        tfIDX = true(1,2);
        tfIDX(idx) = false;

        tf_si = si >= 0 & si <= 1;
        if any(tf_si)
            tf_Pj = Pj(tfIDX, tf_si) == Aj(tfIDX)*si(tf_si) + Bi(tfIDX);
            if any(tf_Pj)
                intersection = true;
                tmp_si = si(tf_si);
                xy = [xy, Ci*[tmp_si(tf_Pj); ones(1, nnz(tf_Pj))]];
            end
        end

    end
    cleanxy = [];
    for i = 1:size(xy, 2)
        if isempty(cleanxy) || ~any(all(abs(cleanxy - xy(:, i)) < 1e-10, 1))
            cleanxy(:, end+1) = xy(:, i);
        end
    end
    xy = cleanxy;

end