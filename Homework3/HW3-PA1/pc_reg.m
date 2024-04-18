function [Tba] = pc_reg(as, bs)
%PC_REG This function computes the transform between 2 sets of 3D points
%(point cloud registration)
%   Inputs:
%       as, bs = Nx3 matrices where N is the number of points and each
%       row is the xyz coordinates of one point. Sets must be
%       corresponding, ie row i of as must represent the same point as row
%       i of bs
%   Outputs:
%       Tba = 4x4 transformation matrix from point set a to b

% Validate inputs
% Check that as and bs are same size
if ~all(isequal(size(as), size(bs)), 'all')
    error("Inputs as and bs are of different sizes");
end

% N = number of points
N = size(as, 1);

% Find centroids of a and b
a_cent = sum(as, 1) / N;
b_cent = sum(bs, 1) / N;

% Find error vectors
a_err = as - a_cent;
b_err = bs - b_cent;

% Compute H-matrix for SVD
H = zeros(3, 3);
for i = 1:N
    Hi = [a_err(i, 1) * b_err(i, 1), a_err(i, 1) * b_err(i, 2), a_err(i, 1) * b_err(i, 3);
          a_err(i, 2) * b_err(i, 1), a_err(i, 2) * b_err(i, 2), a_err(i, 2) * b_err(i, 3);
          a_err(i, 3) * b_err(i, 1), a_err(i, 3) * b_err(i, 2), a_err(i, 3) * b_err(i, 3)];
    H = H + Hi;
end

% Singular value decomposition
[U, S, V] = svd(H);

% Compute R
R = V * U';

% Verify that det(R) = 1
% Else, use alternate R
if ~ismembertol(det(R), 1, 1e-4)
    V_prime = [V(:, 1), V(:, 2), -V(:, 3)];
    R = V_prime * U';
    if ~ismembertol(det(R), 1, 1e-4)
        error("SOL");
    end
end
%}

%{
% Quaternion approach
M = zeros(4 * N, 4);
for i = 1:N
    Mi = [0, (b_err(i, :) - a_err(i, :));
          (b_err(i, :) - a_err(i, :))', v2skew(b_err(i, :) + a_err(i, :))];
    M(1 + 4 * (i - 1) : 4 * i, :) = Mi;
end

% SVD of M to get q
[~, ~, V] = svd(M);
q = V(:, 4);

% Quaternion to R
R = quat2r(q');
%}

% Calculate p
p = b_cent' - R * a_cent';

% Return transform T = (R, p)
Tba = [R, p; [0 0 0 1]];

% Validate outputs
% Tba * a = b
% as_homo = [as'; ones(1, N)];
% bs_homo = Tba * as_homo;
% bs_test = bs_homo(1:3, :)';
% err = bs - bs_test;
% avg_err = mean(abs(err), "all");
% max_err = max(abs(err), [], "all");
% err_v = [avg_err; max_err];

end