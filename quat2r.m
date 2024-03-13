function [R] = quat2r(quat)
%QUAT2R Converts quaternion to rotation matrix
%   Inputs:
%       quat = 1x4 vector representing unit quaternion with structure:
%           [q0, q1, q2, q3], where Q = q0 + q1*i + q2*j + q3*k
%   Outputs:
%       R = 3x3 rotation matrix representation of quat, SO(3)

    q0 = quat(1);
    q_vector = quat(2:4);

    % Validate inputs
    % Unit quaternion has a magnitude of 1
    tol = 1e-4;
    if ~isequal(size(quat), [1 4]) || ~ismembertol(norm(quat), 1, tol)
        error("Input quat is not a valid unit quaternion")
    end

    % First convert to axis-angle form
    theta = 2 * acos(q0);
    if theta == 0
        w = [0 0 0];
    else
        w = q_vector / sin(theta / 2);
    end
    axang = [w theta];

    % Then convert axis-angle to rotation matrix
    R = axisangle2r(axang);
end