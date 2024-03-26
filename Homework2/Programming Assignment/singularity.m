function tf = singularity(JacobianMatrix)
%SINGULARITY Analyztically calculates the singularity configurations of a
%robot based on the Jacobian matrix provided
%   Inputs:
%       J = 6xn Jacobian of current robot position
%   Outputs:
%       tf = Boolean, true if robot is in a singularity

    rankJ = rank(JacobianMatrix);     % calculate the rank of the jacobian matrix
    n = length(JacobianMatrix);     % calculate how many jacobian columns there are

    % determine if the jacobian matrix has a rank deficiency
    if rankJ < n       % if rank of jacobian is less than jacobian columns
        disp('Jacobian matrix has a rank deficiency:');
        disp('Robot is in a singularity configuration.');
        tf = true;
    else    % if rank of jacobian is equal to jacobian columns
        disp('Robot is not in a singularity configuration.');
        tf = false;
    end
end