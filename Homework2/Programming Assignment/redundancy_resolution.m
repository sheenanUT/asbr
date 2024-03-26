% Find joint velocities to maximize manipulability measure and move away from singularities
% Input:
%     Blist: list of screws in {b} frame
%     thetalist: list of joint angles
%     Tbd: transformation matrix of desired pose in {b}
% Output:
%     thVelList: list of joint velocities
function thVelList = redundancy_resolution(Blist, thetalist, Tbd)
    difference = 1e-6;
    k0 = 1;        % kinematic constant
    q = thetalist;      % q_i
    q_next = zeros(1, length(q));       % q_i+1

    % find v and w from Tbd
    [B_se3, th] = SE3log(Tbd);      % screw in body frame in se(3) form
    Vb = se3toR6(B_se3);        % twist in {b} in R^6 form

    % generate q_i+1
    for n = 1:length(q)
        q_next(n) = q(n) + difference;
    end

    % find jacobian of q_i and q_i+1
    J = J_body(Blist, q);      % calculate jacobian matrix | J
    J_next = J_body(Blist, q_next);     % J_i+1
    
    % manipulability measure
    w_q = sqrt(det(J * J'));        % w(q)^i
    w_q_next = sqrt(det(J_next * J_next'));     % w(q)^i+1


    dw_dq = zeros(1, length(q_next));
    % calculate differentiation
    % w(q)^i+1 - w(q) / q_i+1 - q_i
    for n = 1:length(q_next)
        dw_dq(n) = (w_q_next-w_q) / (q_next(n)-q(n));
    end

    q0_dot = k0*dw_dq';     % calculate convenient joint velocities
    % calculate proper joint velocities
    q_dot = JacobianPInv(J)*Vb + (eye(length(q0_dot)) - JacobianPInv(J)*J) * q0_dot;

    thVelList = q_dot;
end