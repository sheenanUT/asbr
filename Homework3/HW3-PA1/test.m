% [Cs, post_EM, post_opt] = read_output('pa1-debug-a-output1.txt');
% disp(post_EM);

% [t_g, P_dimple] = EM_pivot('pa1-debug-a-empivot.txt');
% t_g
% P_dimple
filename_opt = 'pa1-debug-a-optpivot.txt';
filename_body = "pa1-debug-a-calbody.txt";
[t_g, P_dimple] = opt_pivot(filename_opt, filename_body);
% [ds, ~, ~] = read_calbody("pa1-debug-a-calbody.txt");
% [Ds, Hs] = read_optpivot(filename);