%% Robot Specific Variables
% test variables
% syms L1 L2 L3 L4 L5 L6
% syms th1 th2 th3 th4 th5 th6

% link dimensions
L1 = 500 * 10^-3;
L2 = 250 * 10^-3;
L3 = 770 * 10^-3;
L4 = 70 * 10^-3;
L5 = 780 * 10^-3;
L6 = 215 * 10^-3;

% joint angles
th1 = 10;
th2 = 20;
th3 = 30;
th4 = 40;
th5 = 50;
th6 = 60;



%% Part a: Find the FK from spatial frame using FK_space.m
% home configuration
M = [
    1, 0, 0, L2+L3+L5+L6;
    0, 1, 0, 0;
    0, 0, 1, -L4;
    0, 0, 0, 1];

% axes of rotation
w1 = [0; 0; -1];
w2 = [0; 1; 0];
w3 = [0; 1; 0];
w4 = [-1; 0; 0];
w5 = [0; 0; 1];
w6 = [-1; 0; 0];

% locations of center of rotations
q1 = [0; 0; 0];
q2 = [L2; 0; 0];
q3 = [L2+L3; 0; 0];
q4 = [0; 0; -L1];
q5 = [L2+L3+L5; 0; -L4];
q6 = [0; 0; -L4];

w_list = [w1, w2, w3, w4, w5, w6];  % list of omegas
q_list = [q1, q2, q3, q4, q5, q6];  % list of q's
th_list = [th1, th2, th3, th4, th5, th6];   % list of joint angles

v_list = [];    % list of velocity vectors
% compute cross products of w and q. Add them to v_list
for i = 1:length(w_list)
    v = cross(w_list(:, i), q_list(:, i));  % compute the cross product
    v_list = [v_list, v];   % add velocity vector element to v_list
end

screw_list = [];
% format the screw axes using w and v. Add them to screw_list
for i = 1:length(w_list)
    screw = [w_list(:, i ); v_list(:, i)];  % stack w(i) and v(i)
    screw_list = [screw_list, screw];   % add screw axis to screw_list
end

% calculate and display the spacial forward kinematics
T_sb = FK_space(M, screw_list, th_list);
disp(T_sb);



%% Part c: Find the forward kinematics in reference to the body frame | FK_body Function
T_bs = FK_body(M, screw_list, th_list);
disp(T_bs);