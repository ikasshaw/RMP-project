% bench_subdivide.m
% Compare: original subdivideRectangle (yours) vs. cell-opt and fast numeric versions.

clear; clc; rng(0);

% --- config ---
N = 1e5;                           % number of calls per method (adjust as needed)

% --- sanity: make sure your original is on path ---
assert(exist('subdivide','file')==2, ...
    'Your original subdivideRectangle.m must be on the MATLAB path.');

% --- test data: random 2x4 inputs (order doesn't matter for timing) ---
R = rand(2,4,N);

% --- warm-up JIT ---
subdivideRectangle(R(:,:,1));
subdivideRectangle_cell_opt(R(:,:,1));
subdivideRectangle_fast(R(:,:,1));

% --- benchmark ---
t_orig = timeit(@() run_many(@subdivideRectangle,          R));
t_cell = timeit(@() run_many(@subdivideRectangle_cell_opt, R));
t_fast = timeit(@() run_many(@subdivideRectangle_fast,     R));

% --- report ---
fprintf('N = %d calls per method\n', N);
fprintf('Original (cell)       : %7.3f s   (%8.0f calls/s)\n', t_orig, N/t_orig);
fprintf('Cell-optimized (vec)  : %7.3f s   (%8.0f calls/s)\n', t_cell, N/t_cell);
fprintf('Numeric 3D (fastest)  : %7.3f s   (%8.0f calls/s)\n', t_fast, N/t_fast);

% ===== helpers =====
function run_many(f, R)
    Nloc = size(R,3);
    for i = 1:Nloc
        f(R(:,:,i));              % discard output; still exercises allocations/work
    end
end

function S = subdivideRectangle_rect_fast(R, S)
% R: 2x4 corners of a RECTANGLE in cyclic order (rotated is fine).
% S: optional preallocated 2x4x4 array (in-place fill). If omitted, allocated.

    if nargin < 2
        S = zeros(2,4,4,'like',R);
    end

    % Center via opposite corners (rectangle property)
    C = 0.5 * (R(:,1) + R(:,3));

    % Adjacent midpoints
    M  = 0.5 * (R + R(:,[2 3 4 1]));
    Mp = M(:,[4 1 2 3]);  % previous-edge midpoints

    % Fill 4 sub-rectangles; no permute/repmat
    S(:,:,1) = [R(:,1)  M(:,1)  C  Mp(:,1)];
    S(:,:,2) = [R(:,2)  M(:,2)  C  Mp(:,2)];
    S(:,:,3) = [R(:,3)  M(:,3)  C  Mp(:,3)];
    S(:,:,4) = [R(:,4)  M(:,4)  C  Mp(:,4)];
end


% Extra warm-up
subdivideRectangle_rect_fast(R(:,:,1));

% Preallocate once for in-place timing
Sbuf = zeros(2,4,4,'like',R);

t_rect  = timeit(@() run_many(@subdivideRectangle_rect_fast, R));                 % allocates each call
t_rectI = timeit(@() run_many_inplace(@subdivideRectangle_rect_fast, R, Sbuf));   % reuses buffer

fprintf('Rectangle-fast (alloc) : %7.3f s   (%8.0f calls/s)\n', t_rect,  N/t_rect);
fprintf('Rectangle-fast (inplc) : %7.3f s   (%8.0f calls/s)\n', t_rectI, N/t_rectI);

function run_many_inplace(f, R, Sbuf)
    Nloc = size(R,3);
    for i = 1:Nloc
        f(R(:,:,i), Sbuf);
    end
end
