function JPinv = JacobianPInv(JacobianMatrix)
%JPINV Calculates the pseudoinverse of a jacobian matrix if m~=n
%   Input:
%       JacobianMatrix: Jacobian matrix
%   Output:
%       JPinv = pseudoinverse of JacobianMatrix

    % rows and columns of matrix | m: rows, n: columns
    [m, n] = size(JacobianMatrix);
    J = JacobianMatrix;
    Jt = transpose(JacobianMatrix);

    if m == n && det(J) ~= 0
        JPinv = inv(J);        % inverse of jacobian matrix
    elseif n > m || det(J) == 0
        JPinv = Jt * inv(J*Jt);
    elseif n < m
        JPinv = inv(Jt*J) * Jt;
    end

end