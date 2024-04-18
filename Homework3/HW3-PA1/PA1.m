% [Cs, post_EM, post_opt] = read_output("pa1-debug-a-output1.txt")
% should be 8
[b_tip, b_post] = pivot_calibration(get_FDs('e'));