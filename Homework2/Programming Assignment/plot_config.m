function [] = plot_config(T)
%PLOT_CONFIG Plots an object's configuration in 3D space
%   Inputs:
%       T = 4x4 transformation matrix representing configuration

% Validate inputs: check that T is a 4x4 matrix
% Subfunctions handle further validation steps
if ~is_transform(T)
    error("Input T is not a valid transformation matrix");
end

R = T(1:3, 1:3);
q = T(1:3, 4);
plot_frame(R, q');
end