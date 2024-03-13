function [tf] = is_rotation_matrix(R)
%IS_ROTATION_MATRIX Checks whether input is a SO(3) rotation matrix
%   Inputs:
%       R = input matrix
%   Outputs:
%       tf = boolean, true if R is a rotation matrix
%   R is a rotation matrix if all the following are true:
%       1. R is 3x3
%       2. R'R = RR' = I
%       3. det(R) = 1

tf = true;

% Check if R is 3x3
    if ~isequal(size(R), [3 3])
        tf = false;
    % Check properties of rotation matrix
    elseif ~ismembertol(R * R.', eye(3), 1e-4)...       % RR' = I
            | ~ismembertol(R.' * R, eye(3), 1e-4)...    % R'R = I
            | ~ismembertol(det(R), 1, 1e-4) %#ok<OR2>   % det(R) = 1
        tf = false;
    end
end