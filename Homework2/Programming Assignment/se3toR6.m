% Converts 4x4 matrix in se3 form to 1x6 R^6 form.
% Mostly used for screw or twist vectors.
% Input:
%   se3: 4x4 matrix in se(3) form | screw or twist
% Ouput:
%   1x6 matrix in R^6 form | screw or twist
function R6 = se3toR6(se3)
    w_t = skew2v(se3(1:3, 1:3));      % omega vector
    v = se3(1:3, 4);        % velocity vector
    w = transpose(w_t);
    
    % twist or scew vector
    R6 = [
        w;
        v;
    ];
end