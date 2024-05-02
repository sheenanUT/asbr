function qd = q_desired_a(Tsb, J, p_tip, p_goal, th_list_current)
    [A, d] = opt_distance(Tsb, J, p_tip, p_goal);       % generate A and d in (Ax-d) = 0 ---> Ax = d

    C = A;
    b = d+3;        % set b to d+3 | (Ax-d) <= 3 --> Ax<= d+3

    % find lower and upper bound
    lower_bound = ([-180; -145; -130; -350; -125; -350] * pi/180) - th_list_current';        % set lower limit of joint angles in radians
    upper_bound = ([180; 45; 150; 350; 125; 350] * pi/180) - th_list_current';       % set upper limit of joint angles in radians

    % perform least sqaure problem with lower bound and upper bound
    [qd,resnorm,residual,exitflag,output,lambda] = lsqlin(C, d, A, b, [], [], lower_bound, upper_bound);
end