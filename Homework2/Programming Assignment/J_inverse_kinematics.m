%% find the inverse kinematics using an iterative numerical algorithm
% Input:
    % Tsd: desired pose of end effector in spaceframe
    % thetalist_guess: initial guess of joint angles
    % M: home configuration of end effector
    % Blist: list of screws in body frame
    % Bqlist: screw location vectors in body frame

function thetalistd = J_inverse_kinematics(M, Blist, thetalist_guess, Bqlist, Tsd)
    errw = 0.01;       % error value of angular velocity
    errv = 0.001;       % error value of velocity 
    max = 20;       % max iterations of algorithm

    
    i = 0;
    thetalist = thetalist_guess;
    Tbs = FK_body(M, Blist, thetalist, Bqlist);       % find the FK in body frame
    [S_se3, th] = SE3log(Tbs*Tsd);        % screw in body frame
    Vb = se3toR6(S_se3);
    % err_condition: error condition | ||omega||>errw or ||v||>errv
    err_condition = (norm(Vb(1:3)) > errw) || (norm(Vb(4:6)) > errv);

    % iterative inverse kinematic algorithm
    %   Condition: loop while error condition is true and only [max] times
    while err_condition && i<max
        % set new joint angle list
        thetalist = thetalist + JacobianPInv(J_body(Blist, thetalist)) * Vb;
        Tbs = FKinBody(M, Blist, thetalist);       % find the FK in body frame
        [S_se3, th] = SE3log(Tbs*Tsd);        % screw in body frame
        Vb = se3toR6(S_se3);
        % err_condition: error condition | ||omega||>errw or ||v||>errv
        err_condition = (norm(Vb(1:3)) > errw) || (norm(Vb(4:6)) > errv);

        i = i+1;
    end
    thetalistd = thetalist;
end