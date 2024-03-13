function [skew] = v2skew(v)
%V2SKEW Converts a vector to skew-symmetric matrix representation
%   Inputs:
%       v = 1x3 vector
%   Outputs:
%       skew = 3x3 skew-symmetric matrix representation of v

    % Validate inputs
    if ~all(isequal(size(v), [1 3]))
        error("Input v is not a valid 1x3 vector");
    end
    
    v1 = v(1); v2 = v(2); v3 = v(3);
    skew = [0, -v3, v2;
            v3, 0, -v1;
            -v2, v1, 0];
    
    % Validate outputs
    % Exception for symbolic variables
    if class(skew) ~= "sym" && ~issymmetric(skew, "skew")
        error("Could not calcualte skew symmetric matrix");
    end
end