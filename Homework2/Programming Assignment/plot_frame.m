function [] = plot_frame(R, q)
%PLOT_FRAME Plots reference frame in 3D space
%   Inputs:
%       R = 3x3 SO3 rotation matrix representing frame to plot
%       q = 1x3 vector for origin of R

% Validate inputs: check that R is SO3 and q is a 1x3 vector
if ~is_rotation_matrix(R)
    error("Input R is not a valid SO3 rotation matrix")
elseif ~isequal(size(q), [1, 3])
    error("Input q is not a valid 1x3 vector");
end

hold on % All on same plot
plot_vector(R(:, 1)', q, '-r');  % Plot x
plot_vector(R(:, 2)', q, '-g');  % Plot y
plot_vector(R(:, 3)', q, '-b');  % Plot z

end