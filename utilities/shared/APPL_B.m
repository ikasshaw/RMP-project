% Isaac Shaw
% Robot Motion Planning
% 6/14/2025
% Function to check the Applicability of B in 2-d

function typeB = APPL_B(q, A, B, options)

    arguments
        q double
        A double
        B double
        options.option = false
    end

    % Check that A, B, and q are valid as done
    % checkVertices(A);
    % checkVertices(B);
    % checkQ(q);

    [ma, na] = size(A);
    [mb, nb] = size(B);

    typeB = false(na, nb);

    % Reshape A and B with additional rows for matrix operations below
    A(3,:) = 0;
    A(4,:) = 1;
    B(3,:) = 0;

    % Rotate A into the configuration defined by q
    H = Rz(q(3));
    Aq = H*A;
    Aq = Aq(1:3, :); % Drop the bottom row of 1s

    z_unit = [0; 0; 1];

    for j=1:nb

        b_j = B(:, j);

        if j == nb
            b_next = B(:,1);
        else
            b_next = B(:,j+1);
        end

        E_B = b_next - b_j;

        V_jB = cross((E_B), z_unit);

        for i = 1:na

            a_i = Aq(:, i);

            if i == 1
                a_prev = Aq(:,na);
                a_next = Aq(:,i+1);
            elseif i == na
                a_prev = Aq(:,i-1);
                a_next = Aq(:,1);
            else
                a_prev = Aq(:,i-1);
                a_next = Aq(:,i+1);
            end

            E_A_prev = a_prev - a_i;
            E_A_next = a_next - a_i;

            typeB(i,j) = dot((E_A_prev), V_jB) >= 0 && dot((E_A_next), V_jB) >= 0;

        end

    end
    
end