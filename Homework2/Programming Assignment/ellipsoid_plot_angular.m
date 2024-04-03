function [semi_axes] = ellipsoid_plot_angular(J, T)
%ELLIPSOID_PLOT_LINEAR Calculates & plots angular manipulability ellipsoid
%of robot
%   Inputs:
%       J = 6xn Jacobian matrix (space or body)
%       T = 4x4 configuration of end effector
%   Outputs:
%       semi_axes = 3x3 matrix where each column is one of the semi-axes
%       of the ellipsoid

% Validate inputs
% J should have 6 rows
if size(J, 1) ~= 6
    error("Input J is not a valid Jacobian matrix");
% T must be a valid transformation
elseif ~is_transform(T)
    error("Input T is not a valid transformation matrix");
end

Jw = J(1:3, :);    % Angular Jacobian, top 3 rows
A = Jw * Jw';   % 3x3 matrix
[e_vecs, e_vals] = eig(A);  % Eigenvals/vecs of A

% Treat e_vecs as a rotation matrix for the ellipsoid
if det(e_vecs) + 1 < 1e-4    % Flip from LHR to RHR if necessary
    e_vecs = [e_vecs(1, :);
              e_vecs(3, :);
              e_vecs(2, :)];
end

% Get ZYX Euler angles from e_vecs
zyx = r2zyx(e_vecs) * 180/pi;   % Convert to deg

% Plot ellipsoid
axis equal;
hold on;

q = T(1:3, 4);  % End-effector position
[X, Y, Z] = ellipsoid(q(1), q(2), q(3), sqrt(e_vals(1, 1)),...
          sqrt(e_vals(2, 2)), sqrt(e_vals(3, 3)));
s = surf(X, Y, Z, "FaceAlpha", 0.5);

% Rotate ellipsoid to correct orientation
rotate(s, [1 0 0], zyx(3), q);
rotate(s, [0 1 0], zyx(2), q);
rotate(s, [0 0 1], zyx(1), q);

% Plot major axes, for comparison
for i = 1:3
    plot_vector(e_vecs(:, i)' * sqrt(e_vals(i, i)), q', '-ko')
end
hold off;

% Return semi-axes
semi_axes = zeros([3 3]);
for i = 1:3
    semi_axes(:, i) = e_vecs(:, i) * sqrt(e_vals(i, i));
end