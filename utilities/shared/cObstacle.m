function CB = cObstacle(theta, A, B)

    q = [0; 0; theta];
    typeA = APPL_A(q, A, B);
    typeB = APPL_B(q, A, B);

    [ma, na] = size(A);
    [mb, nb] = size(B);

    CB = [];

    % Reshape A with additional rows for matrix operations below
    A(3,:) = 0;
    A(4,:) = 1;

    % Rotate A into the configuration defined by q
    H = Rz(q(3));
    Aq = H*A;
    Aq = Aq(1:2, :); % Drop the bottom rows of 0s and 1s

    for i=1:na

        a_i = Aq(:, i);

        if i == na
            a_next = Aq(:,1);
        else
            a_next = Aq(:,i+1);
        end

        for j = 1:nb

            b_j = B(:, j);

            if j == nb
                b_next = B(:,1);
            else
                b_next = B(:,j+1);
            end

            if typeA(i, j)
                CB(:, end+1) = b_j - a_i;
                CB(:, end+1) = b_j - a_next;
            end

            if typeB(i, j)
                
                CB(:, end+1) = b_j - a_i;
                CB(:, end+1) = b_next - a_i;
            end
        
        end
    end
    k = convhull(CB');
    k(end) = [];

    CB = CB(:,k);

end