function pivot_calibration(F_k, b_tip, b_post)
    R_k = F_k(1:3, 1:3);        % find the rotation matrix of F_k
    p_k = F_k(4, 1:3);      % find the translation vector of F_k
    
end