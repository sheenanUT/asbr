function [adj] = adjTransform(T)
%ADJTRANSFORM Calculates adjunct representation of given transformation
%   Inputs:
%       T = 4x4 rigid body transformation matrix
%   Outputs:
%       adj = 6x6 adjunct matrix representation of T

R = T(1:3, 1:3);  % Rotation matrix
p = T(1:3, 4)';    % Translation vector

adj = [R, zeros([3 3]);
        v2skew(p) * R, R];

end