% Get transform from config
function H = qToH(q)
    X = q(1:2, :);
    k = q(3);
    R = [cos(k) -sin(k); sin(k) cos(k)];
    H = eye(4,4);
    H(1:2, 1:2) = R;
    H(1:2,4) = X;
end
