function qd = q_desired(Tsb, J, p_tip, p_goal)
    [qd, flag] = lsqr(@(delta_q) opt_distance(Tsb, J, p_tip, p_goal, delta_q), 0);

    
end