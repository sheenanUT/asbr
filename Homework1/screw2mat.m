function [mat] = screw2mat(screw)
%SCREW2MAT Converts screw axis vector to skew symmetric matrix
%representation
%   Inputs:
%       screw = 1x6 vector of screw axis with structure:
%           [w1 w2 w3 v1 v2 v3]
%   Outputs:
%       mat = 4x4 matrix representation of screw

    % Validate inputs
    if ~all(isequal(size(screw), [1 6]))
        error("Input screw is not a valid 1x6 vector");
    end
    
    w = screw(1:3);
    v = screw(4:6);
    w_skew = v2skew(w);

    % Accomodate symbolic inputs
    if class(screw) == "sym"
        mat = sym(zeros([4 4]));
    else
        mat = zeros([4 4]);
    end

    mat(1:3, 1:3) = w_skew;
    mat(1:3, 4) = v;

end