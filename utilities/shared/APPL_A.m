% Isaac Shaw
% Robot Motion Planning
% 6/14/2025
% Function to check the Applicability of A in 2-d

function typeA = APPL_A(q, A, B)

    arguments
        q double
        A double
        B double
    end

    % Check that A, B, and q are valid as done
    % checkVertices(A);
    % checkVertices(B);
    % checkQ(q);

    [ma, na] = size(A);
    [mb, nb] = size(B);

    typeA = false(na, nb);

    % Reshape A and B with additional rows for matrix operations below
    A(3,:) = 0;
    A(4,:) = 1;
    B(3,:) = 0;

    % Rotate A into the configuration defined by q
    H = Rz(q(3));
    Aq = H*A;
    Aq = Aq(1:3, :); % Drop the bottom row of 1s

    z_unit = [0; 0; 1];

    for i=1:na

        a_i = Aq(:, i);

        if i == na
            a_next = Aq(:,1);
        else
            a_next = Aq(:,i+1);
        end

        E_A = a_next - a_i;
        V_iA = cross((E_A), z_unit);

        for j = 1:nb

            b_j = B(:, j);

            if j == 1
                b_prev = B(:,nb);
                b_next = B(:,j+1);
            elseif j == nb
                b_prev = B(:,j-1);
                b_next = B(:,1);
            else
                b_prev = B(:,j-1);
                b_next = B(:,j+1);
            end

            E_B_prev = b_prev - b_j;
            E_B_next = b_next - b_j;

            typeA(i,j) = dot(V_iA, (E_B_prev)) >= 0 && dot(V_iA, (E_B_next)) >= 0;
        end
    end
end