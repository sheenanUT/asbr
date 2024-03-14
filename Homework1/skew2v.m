function [v] = skew2v(skew)
%SKEW2V Converts a skew-symmetric matrix to its vector representation
%   Inputs:
%       skew = 3x3 skew-symmetric matrix
%   Outputs:
%       v = 1x3 vector representation of skew

% Validate inputs
if ~issymmetric(skew, "skew") || ~all(isequal(size(skew), [3 3]))
    error("Input skew is not a valid 3x3 skew symmetric matrix");
end

v1 = skew(3, 2);
v2 = skew(1, 3);
v3 = skew(2, 1);
v = [v1 v2 v3];