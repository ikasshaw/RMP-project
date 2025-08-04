% Isaac Shaw
% Robot Motion Planning
% 7/26/2025

function S = subDivRec(R)
% function [S, new_adjacencies] = subDivRec(R, adjacencies)

    arguments
        R (2, 4) double
    end

    % Input validation
    % Check that points define convex hull in counter-clockwise order
    k = convhull(R');
    k(end) = [];
    if numel(k) ~= 4 && ~isequal(k.',1:n)
        error('Input R must define the points of a rectangle')
    end

    % Edge and diagonal lengths
    V = diff([R R(:,1)], 1, 2);
    L = sqrt(sum(V.^2,1));
    D = sqrt(sum([R(:,1) - R(:,3), R(:,1) - R(:,3)].^2,1));

    % Scale-aware tolerance
    tol = 1e-9;

    % Rectangle opposite sides and diagonals must be equal length
    if abs(L(1) - L(3)) > tol || abs(L(2) - L(4)) > tol || D(1) - D(2) > tol
        error('Input points do not define a rectangle')
    end

    % Center point of rectangle
    C = 0.5 * (R(:, 1) + R(:, 3));

    % Adjacent midpoints
    M  = 0.5 * (R + R(:,[2 3 4 1]));

    % Fill 4 sub-rectangles and define each starting at lower left working clockwise
    S{1} = [R(:,1), M(:,1), C, M(:,4)];
    S{2} = [M(:,1), R(:,2), M(:,2), C];
    S{3} = [C, M(:,2), R(:,3), M(:,3)];
    S{4} = [M(:,4), C, M(:,3), R(:,4)];
    % S{2} = [R(:,2)  M(:,2)  C  M(:,1)];
    % S{3} = [R(:,3)  M(:,3)  C  M(:,2)];
    % S{4} = [R(:,4)  M(:,4)  C  M(:,3)];

end