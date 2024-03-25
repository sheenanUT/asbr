%% Calculate the pseudoinverse of a jacobian matrix if m~=n
% Input:
%   JacobianMatrix: Jacobian matrix
function JPinv = JacobianPInv(JacobianMatrix)
    % rows and columns of matrix | m: rows, n: columns
    [m, n] = size(JacobianMatrix);
    J = JacobianMatrix;
    Jt = transpose(JacobianMatrix);

    if m == n
        JPinv = inv(J);        % inverse of jacobian matrix
    elseif n > m
        JPinv = Jt * inv(J*Jt);
    elseif n < m
        JPinv = inv(Jt*J) * Jt;
    end

end