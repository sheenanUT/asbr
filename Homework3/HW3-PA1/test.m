[Cs, post_EM, post_opt] = read_output('pa1-debug-a-output1.txt');
disp(post_EM);

[t_g, P_dimple] = EM_pivot('pa1-debug-a-empivot.txt');
t_g
P_dimple

%% chaange pc_reg!!