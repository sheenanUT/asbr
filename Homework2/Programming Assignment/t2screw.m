function [screw, theta] = t2screw(T)
%T2SCREW Converts transformation matrix to screw axis and angle
%   Inputs:
%       T = 4x4 transformation matrix
%   Outputs:
%       screw = 6x1 screw axis vector
%       theta = scalar rotation angle, rad
%   [screw * theta] transforms from origin to T

% Input validation
% T must be transformation matrix
if ~is_transform(T)
    error("Input T is not a valid transformation matrix");
end

R = T(1:3, 1:3);
p = T(1:3, 4);

% Matrix logarithm of T
if all(ismembertol(R, eye(3)))
    w = [0; 0; 0];
    v = p / norm(p);
    theta = norm(p);
else
    axang = r2axisangle(R);
    w = axang(1:3)';
    theta = axang(4);
    w_skew = v2skew(w');
    v = (1 / theta * eye(3) - 1/2 * w_skew + (1 / theta - 1/2 * cot(theta / 2)) * w_skew^2) * p;
end

screw = [w; v];
end