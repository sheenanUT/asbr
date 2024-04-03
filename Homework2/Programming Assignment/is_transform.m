function tf = is_transform(T)
%IS_TRANSFORM Checks whether input is a rigid-body transformation matrix
%   Inputs:
%       T = input matrix
%   Outputs:
%       tf = boolean, true if T is a transformation matrix
%   T is a transformation matrix if:
%       1. T is 4x4
%       2. R within T is an SO3 rotation matrix
%       3. The bottom row is [0 0 0 1]

tf = true;
% Check dimension of T
if ~isequal(size(T), [4 4])
    tf = false;
% Check rotation matrix
elseif ~is_rotation_matrix(T(1:3, 1:3))
    tf = false;
% Check bottom row
elseif ~isequal(T(4, :), [0 0 0 1])
    tf = false;
end