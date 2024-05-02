function qd = q_desired_b(Tsb, J, p_tip, p_goal)

    %% Find (1) Ax=b
    % t = position vector of tool tip in space frame
    R = Tsb(1:3, 1:3);
    z = [0; 0; 1];
    x = Tsb * p_tip;

    % Ja = angular Jacobian
    % Je = linear Jacobian
    Ja = J(1:3, :);
    Je = J(4:6, :);

    % Rearrange distance formula into Ax = b form
    A_1 = -v2skew(t) * Ja + Je;
    d_1 = p_goal - x;

    C_1 = A_1;
    b_1 = d_1+3;        % set b to d+3 | (Ax-d) <= 3 --> Ax<= d+3

    

    %% Find (2) Ax=b
    A_2 = -v2skew(R*z) * Ja;
    b_2 = [0; 0; 0];

    d_2 = b_2;
    C_2 = A_2;


    %% setup lsqin
    % find lower and upper bound
    lower_bound = ([-180; -145; -130; -350; -125; -350] * pi/180) - th_list_current;        % set lower limit of joint angles in radians
    upper_bound = ([180; 45; 150; 350; 125; 350] * pi/180) - th_list_current;       % set upper limit of joint angles in radians

    A = [A_1; A_2];
    b = [b_1; b_2];
    C = [C_1; C_2];
    d = [d_1; d_2];

    % perform least sqaure problem with lower bound and upper bound
    [qd, flag] = lsqlin(C, d, A, b, [], [], lower_bound, upper_bound);
end