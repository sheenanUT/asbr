% matrix log of SE(3) or special euclidean matrices to lie algebra form se(3)
% without th constant
% T -> [S]
% Input:
%     T: transformation matrix
% Output:
%   se3matrix: 4x4 matrix in se3 form
%   th: angle of rotation
function [se3matrix,th] = SE3log(T)
    % pose
    R = T(1:3, 1:3);       % rotation matrix of T
    p = T(1:3, 4);     % location vector of T

    th = acos(1/2 * (trace(R)-1));       % identify angle of rotation
    w_skew = (1 / (2*sin(th))) * (R - transpose(R));      % identify angular velocity in se(3) form
    w_t = skew2v(w_skew);     % w in SO(3) form | 3x1 vector
    w = transpose(w_t);

    % calculate conversion of p to velocity
    invG = (1/th)*eye(3) - (1/2)*w_skew + ((1/th) - ((1/2)*cot(th/2))) * (w_skew)^2;
    velocity = invG*p;

    se3matrix = [
        w_skew, velocity;
        zeros(1, 4);
    ];

end
