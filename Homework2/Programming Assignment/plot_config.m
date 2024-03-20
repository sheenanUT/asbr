function [] = plot_config(T)
%PLOT_CONFIG Plots an object's configuration in 3D space
%   Inputs:
%       T = 4x4 transformation matrix representing configuration

% Validate inputs: check that T is a 4x4 matrix
% Subfunctions handle further validation steps
if ~isequal(size(T), [4, 4])
    error("Input T is not a valid 4x4 matrix");
end

R = T(1:3, 1:3);
q = T(1:3, 4);
plot_frame(R, q');
end