function [axang] = r2axisangle(R)
%R2AXISANGLE Converts rotation matrix to axis-angle form
%   Inputs:
%       R = 3x3 rotation matrix, SO(3)
%   Outputs:
%       axang = 1x4 rotation vector with angle with structure:
%           [wx wy wz theta]
%   R rotates a frame about w by amount theta

    % Validate inputs
    if ~is_rotation_matrix(R)
        error("Input R is not a valid SO(3) rotation matrix");
    end

    % Rotation to axis-angle algorithm
    % If R = I, then theta = 0, w is undefined
    if R == eye(3)
        theta = 0;
        w = [0 0 1];  % Let w = k^, follows convention of base function
    % If trace R = -1, then theta = pi, w is known from formula
    elseif trace(R) == -1
        theta = pi;
        w = 1 / sqrt(2 * (1 + R(3, 3))) * [R(1, 3), R(2, 3), 1 + R(3, 3)];
    else
        theta = acos(1/2 * (trace(R) - 1));
        w_skew = 1 / (2 * sin(theta)) * (R - R.');
        w = skew2v(w_skew);  % Convert skew symmetric matrix to vector
    end
    axang = [w theta];
end