function [vol] = J_volume(J)
%J_VOLUME Calculates manipulability volume (mu3) of Jacobian matrix
%   Inputs:
%       J = 6xn Jacobian matrix
%   Outputs:
%       vol = scalar volume measure
%   Larger values are better.

A = J * J';
vol = sqrt(det(A));
end