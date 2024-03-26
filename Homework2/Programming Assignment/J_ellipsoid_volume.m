function [vol] = J_ellipsoid_volume(J)
%J_ELLIPSOID_VOLUME Calculates manipulability volume (mu3) of Jacobian matrix
%   Inputs:
%       J = 6xn Jacobian matrix
%   Outputs:
%       vol = scalar volume measure
%   Larger values are better.

% Validate inputs
% J should have 6 rows
if size(J, 1) ~= 6
    error("Input J is not a valid Jacobian matrix");
end

A = J * J';
vol = sqrt(det(A));
end