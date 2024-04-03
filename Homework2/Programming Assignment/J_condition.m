function [cond] = J_condition(J)
%J_CONDITION Calculates condition (mu2) of Jacobian matrix
%   Inputs:
%       J = 6xn Jacobian matrix
%   Outputs:
%       cond = scalar condition measure
%   Condition is the square of isotropy. Smaller values are better.

% Validate inputs
% J should have 6 rows
if size(J, 1) ~= 6
    error("Input J is not a valid Jacobian matrix");
end

iso = J_isotropy(J);
cond = iso^2;
end