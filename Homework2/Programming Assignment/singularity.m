%% function analyztically calculates the singularity configurations of a robot based on the jacobian matrix provided
function singularity(JacobianMatrix)        % input is is jacobian matrix of current robot position
    rankJ = rank(JacobianMatrix);     % calculate the rank of the jacobian matrix
    n = length(JacobianMatrix);     % calculate how many jacobian columns there are

    % determine if the jacobian matrix has a rank deficiency
    if rankJ < n       % if rank of jacobian is less than jacobian columns
        disp('Jacobian matrix has a rank deficiency:');
        disp('Robot is in a singularity configuration.');
    else    % if rank of jacobian is equal to jacobian columns
        disp('Robot is not in a singularity configuration.');
    end
end