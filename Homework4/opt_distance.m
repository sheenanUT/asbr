function [D] = opt_distance(Tsb, J, p_tip, p_goal, delta_q)
%OPT_DISTANCE This function finds the distance between the tool tip and
%goal positions for a robot after a change in joint angles
%   Inputs:
%       Tsb = 4x4 transformation matrix from space frame to body frame
%       J = 6xN space-frame Jacobian matrix (N-dof) for robot at current
%       joint positions
%       p_tip = 3x1 vector of tool tip position in body frame (constant)
%       p_goal = 3x1 vector of goal position in space frame
%       delta_q = Nx1 vector of joint angle displacements
%   Outputs:
%       D = distance between tool tip and p_goal after displacing current
%       joint angles by delta_q
%   This function is meant to be used with an optimizer to find the delta_q
%   that minimizes D subject to constraints
%   Formulas obtained from lecture slides 14-2

% Validate inputs
if ~is_transform(Tsb)
    error("Input Tsb is not a valid transformation matrix")
end

N = length(delta_q);
if ~all(isequal(size(J), [6, N]))
    error("The dimensions of inputs J and delta_q do not match")
end

% t = position vector of tool tip in space frame
t_homo = Tsb * [p_tip; 1];  % homogeneous form
t = t_homo(1:3);

% alpha = rotation vector induced by delta_q
% epsilon = translation vector induced by delta_q
v = J * delta_q;
alpha = v(1:3);
epsilon = v(4:6);

% Linearized distance formula for small transformations
D = cross(alpha, t) + epsilon + t - p_goal;

end