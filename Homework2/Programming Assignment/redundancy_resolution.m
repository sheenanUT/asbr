function thVelList = redundancy_resolution(Blist, thetalist, Tbd)
%REDUNDANCY_RESOLUTION Find joint velocities to maximize manipulability
%measure and move away from singularities
%   Inputs:
%       Blist: 6xn list of screws in {b} frame
%       thetalist: 1xn list of joint angles
%       Tbd: 4x4 transformation matrix of desired pose in {b}
%   Output:
%       thVelList: 1xn list of joint velocities
    
    n = length(thetalist);

    % Validate inputs
    % Blist must be 6xn
    if ~isequal(size(Blist), [6 n])
        error("Input Blist is not a 6xn matrix");
    % thetalist_guess must be 1xn
    elseif ~isequal(size(thetalist), [1 n])
        error("Input thetalist is not a 1xn vector");
    % Tbd must be a transformation matrix
    elseif ~is_transform(Tbd)
        error("Input Tbd is not a valid transformation matrix");
    end

    difference = 1e-3;
    k0 = 1;        % kinematic constant
    q = thetalist;      % q_i

    % find v and w from Tbd
    [B_se3, th] = SE3log(Tbd);      % screw in body frame in se(3) form
    Vb = se3toR6(B_se3) * th;       % twist in {b} in R^6 form

    % Get current manipulability
    J = J_body(Blist, q);
    w_q = sqrt(abs(det(J * J')));        % w(q)^i

    % Find dw/dq
    % w(q) is a scalar function of vector q
    % dw/dq = vector [dw/dq1; dw/dq2; ...; dw/dqn]
    dw_dq = zeros(1, length(q));
    for i = 1:length(q)
        % Increment 1 element of q at a time
        q_next = q;
        q_next(i) = q(i) + difference;

        % Get incremented manipulability
        J_next = J_body(Blist, q_next);     % J_i+1
        w_q_next = sqrt(abs(det(J_next * J_next')));     % w(q)^i+1

        % calculate differentiation, finite difference
        % w(q)^i+1 - w(q) / q_i+1 - q_i
        dw_dq(i) = (w_q_next-w_q) / difference;
    end

    q0_dot = k0*dw_dq';     % calculate convenient joint velocities
    % calculate proper joint velocities
    q_dot = JacobianPInv(J)*Vb + (eye(length(q0_dot)) - JacobianPInv(J)*J) * q0_dot;

    thVelList = q_dot;
end