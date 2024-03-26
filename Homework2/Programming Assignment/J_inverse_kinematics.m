function [thetalistd, thVelList] = J_inverse_kinematics(M, Blist,...
                                   thetalist_guess, Bqlist, Tsd)
%J_INVERSE_KINEMATICS finds a robot's inverse kinematics using an iterative
%numerical algorithm, and find optimal joint velocities
%   Input:
%       Tsd: desired pose of end effector in spaceframe
%       thetalist_guess: initial guess of joint angles
%       M: home configuration of end effector
%       Blist: list of screws in body frame
%       Bqlist: screw location vectors in body frame
%   Output:
%       thetalistd: list of joint angles for end-effector to reach desired pose
%       thVellist: list of joint angle velocities to maximize manipulability
%       and move away from singularities
    
    n = length(thetalist_guess);

    % Validate inputs
    % M and Tsd must be transformation matrices
    if ~is_transform(M)
        error("Input M is not a valid transformation matrix");
    elseif ~is_transform(Tsd)
        error("Input Tsd is not a valid transformation matrix");
    % Blist must be 6xn
    elseif ~isequal(size(Blist), [6 n])
        error("Input Blist is not a 6xn matrix");
    % thetalist_guess must be 1xn
    elseif ~isequal(size(thetalist_guess), [1 n])
        error("Input thetalist_guess is not a 1xn vector");
    % Bqlist must be 3xn
    elseif ~isequal(size(Bqlist), [3 n])
        error("Input Bqlist is not a 3xn matrix");
    end

    errw = 1e-4;       % error value of angular velocity
    errv = 1e-4;       % error value of velocity 
    max = 20;       % max iterations of algorithm

    i = 0;
    thetalist = thetalist_guess;

    % find the FK in body frame
    Tbs = FK_body(M, Blist, thetalist, Bqlist, false);

    % Algorithm fails if initial position is singular
    if singularity(J_body(Blist, thetalist))
        error("Cannot calculate IK from singular position");
    end

    % Initial error screw
    [S_se3, th] = SE3log(inv(Tbs)*Tsd);        % screw in body frame
    Vb = se3toR6(S_se3) * th;
    
    % err_condition: error condition | ||omega||>errw or ||v||>errv
    err_condition = (norm(Vb(1:3)) > errw) || (norm(Vb(4:6)) > errv);

    % iterative inverse kinematic algorithm
    %   Condition: loop while error condition is true and only [max] times
    while err_condition && i<max
        % set new joint angle list
        thetalist = thetalist + (JacobianPInv(J_body(Blist, thetalist)) * Vb)';
        Tbs = FK_body(M, Blist, thetalist, Bqlist, false);       % find the FK in body frame
        [S_se3, th] = SE3log(inv(Tbs)*Tsd);        % screw in body frame
        Vb = se3toR6(S_se3) * th;
        % err_condition: error condition | ||omega||>errw or ||v||>errv
        err_condition = (norm(Vb(1:3)) > errw) || (norm(Vb(4:6)) > errv);

        i = i+1;
    end
    thetalistd = thetalist;

    % Redundancy resolution to get joint velocities
    Tbs = FK_body(M, Blist, thetalist_guess, Bqlist, false);
    thVelList = redundancy_resolution(Blist, thetalist_guess, inv(Tbs) * Tsd);
end