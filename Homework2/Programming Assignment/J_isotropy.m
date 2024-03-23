function [iso] = J_isotropy(J)
%J_ISOTROPY Calculates isotropy (mu1) of Jacobian matrix
%   Inputs:
%       J = 6xn Jacobian matrix
%   Outputs:
%       iso = scalar isotropy measure
%   Isotropy measures the extent to which it is equally easy for a robot to
%   move in any direction. Smaller values are better.

A = J * J';
e_vals = eig(A);

% Isotropy = sqrt of max eigenvalue / min eigenvalue
iso = sqrt(max(e_vals) / min(e_vals));

end