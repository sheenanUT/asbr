function [Tab] = pc_reg(as, bs)
%PC_REG This function computes the transform between 2 sets of 3D points
%(point cloud registration)
%   Inputs:
%       as, bs = Nx3 matrices where N is the number of points and each
%       row is the xyz coordinates of one point. Sets must be
%       corresponding, ie row i of as must represent the same point as row
%       i of bs
%   Outputs:
%       Tab = 4x4 transformation matrix from point set a to b

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
[U, ~, V] = svd(H);

% Compute R
R = V * U';

% Verify that det(R) = 1
% If not, we need to do more work
% Hopefully this never happens
if ~ismembertol(det(R), 1, 1e-4)
    error("Red Alert: det(R) != 1");
end

% Calculate p
p = b_cent' - R * a_cent';

% Return transform T = (R, p)
Tab = [R, p; [0 0 0 1]];

end