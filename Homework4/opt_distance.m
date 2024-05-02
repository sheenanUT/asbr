function [A, b] = opt_distance(Tsb, J, p_tip, p_goal)
%OPT_DISTANCE This function sets up the least-squares solver for the joint
%displacements
%   Inputs:
%       Tsb = 4x4 transformation matrix from space frame to body frame
%       J = 6xN space-frame Jacobian matrix (N-dof) for robot at current
%       joint positions
%       p_tip = 3x1 vector of tool tip position in body frame (constant)
%       p_goal = 3x1 vector of goal position in space frame
%   Outputs:
%       A = 3xN matrix for left hand side of least-squares problem
%       b = 3x1 vector for right hand side of least-squares problem
%   Formulas obtained from lecture slides 14-2

% Validate inputs
if ~is_transform(Tsb)
    error("Input Tsb is not a valid transformation matrix")
end

% t = position vector of tool tip in space frame
t_homo = Tsb * [p_tip; 1];  % homogeneous form
t = t_homo(1:3);

% Ja = angular Jacobian
% Je = linear Jacobian
Ja = J(1:3, :);
Je = J(4:6, :);

% Rearrange distance formula into Ax = b form
A = -v2skew(t) * Ja + Je;
b = p_goal - t;

end