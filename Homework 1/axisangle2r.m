function [R] = axisangle2r(axang)
%AXISANGLE2R Converts axis angle rotation to rotation matrix
%   Inputs:
%       axang = 1x4 vector of axis angle representation with structure:
%           [w1, w2, w3, theta]
%   Outputs:
%       R = 3x3 rotation matrix representation of axang, SO(3)
%   R rotates a frame about w by amount theta

    w = axang(1:3);
    theta = axang(4);

    % Validate inputs
    % Check if w is a 1x3 vector with magnitude 1 or 0
    tol = 1e-4;
    if ~isequal(size(w), [1 3])||...
            (~ismembertol(norm(w), 1, tol) && ~ismembertol(norm(w), 0, tol))
        error("Input w is not a valid rotation vector")
    end

    % Trivial case: if w=0 then R=I
    if ismembertol(norm(w), 0, tol)
        R = eye(3);
    else
    % Non-trivial case
    % R(w, theta) = e^(w^ * theta)
        w_skew = v2skew(w);
        R = expm(w_skew * theta);
    end
end