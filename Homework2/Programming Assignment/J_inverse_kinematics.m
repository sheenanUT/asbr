% find the inverse kinematics using an iterative numerical algorithm
% and Find joint velocities to maximize manipulability measure and move
% away from singularities
% Input:
%   Tsd: desired pose of end effector in spaceframe
%   thetalist_guess: initial guess of joint angles
%   M: home configuration of end effector
%   Blist: list of screws in body frame
%   Bqlist: screw location vectors in body frame
% Output:
%     thetalistd: list of joint angles for end-effector to reach desired pose
%     thVellist: list of joint angle velocities to maximize manipulability
%     and move away from singularities

    % TODO: Return an error message if starting config is singular

function [thetalistd, thVelList] = J_inverse_kinematics(M, Blist, thetalist_guess, Bqlist, Tsd)
    errw = 1e-4;       % error value of angular velocity
    errv = 1e-4;       % error value of velocity 
    max = 20;       % max iterations of algorithm

    i = 0;
    thetalist = thetalist_guess;
    Tbs = FK_body(M, Blist, thetalist, Bqlist, false);       % find the FK in body frame
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