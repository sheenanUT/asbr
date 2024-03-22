function [] = plot_vector(v, q, style)
%PLOT_VECTOR Plots vector in 3D space
%   Inputs:
%       v = 1x3 vector to be plotted
%       q = 1x3 vector for origin of v
%       style = string defining visual style of plotted vector, ie '-k' for
%           a solid black line

% Validate inputs: check that v and q are 1x3 vectors and that style is a
% string
if ~isequal(size(v), [1, 3])
    error("Input v is not a valid 1x3 vector");
elseif ~isequal(size(q), [1, 3])
    error("Input q is not a valid 1x3 vector");
elseif ~isequal(class(style), 'string') && ~isequal(class(style), 'char')
    error("Input style is not a valid string");
end

% Shift origin of v to q
v_shifted = v + q;

% Define plot settings
view(3);    % plot in 3D
axis equal; % equal scaling for axes

% Plot line from q to v+q
plot3([q(1), v_shifted(1)], ...
      [q(2), v_shifted(2)], ...
      [q(3), v_shifted(3)], ...
      style);
end