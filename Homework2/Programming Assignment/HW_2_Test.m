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
th_list = [th1 th2 th3 th4 th5 th6];

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
w_list = [w1 w2 w3 w4 w5 w6];

% Screw axis position vectors, space frame
qs1 = [0; 0; 0];
qs2 = [L2; 0; 0];
qs3 = [L2+L3; 0; 0];
qs4 = [0; 0; -L4];
qs5 = [L2+L3+L5; 0; 0];
qs6 = [0; 0; -L4];
qs_list = [qs1, qs2, qs3, qs4, qs5, qs6];

% Screw axis position vectors, body frame
qb1 = [-L5-L3-L2; 0; 0];
qb2 = [-L5-L3; 0; L4];
qb3 = [-L5; 0; L4];
qb4 = [0; 0; 0];
qb5 = [0; 0; 0];
qb6 = [0; 0; 0];
qb_list = [qb1, qb2, qb3, qb4, qb5, qb6];

% Screw axis velocity vectors
vs_list = [];
vb_list = [];
% Loop to compute cross products and add them to v_list
for i = 1:length(w_list)
    % Compute the cross products
    vs = cross(w_list(:, i), qs_list(:, i));
    vb = cross(w_list(:, i), qb_list(:, i));
    
    % Add the result to v_lists
    vs_list = [vs_list, vs];
    vb_list = [vb_list, vb];
end

% Screw axis vectors
screw_list_s = [];
screw_list_b = [];
% Loop to compute cross products and add them to v_list
for i = 1:length(w_list)
    % Stack w(i) and v(i)
    screw_s = [w_list(:, i ); vs_list(:, i)];
    screw_b = [w_list(:, i); vb_list(:, i)];

    % Add the result to s_lists
    screw_list_s = [screw_list_s, screw_s];
    screw_list_b = [screw_list_b, screw_b];
end

% calculate and display the spacial forward kinematics
T_sb = FK_space(M, screw_list_s, th_list);
disp(T_sb);

%% Part c: Find the forward kinematics in reference to the body frame | FK_body Function
T_bs = FK_body(M, screw_list_b, th_list);
disp(T_bs);

%% 
% Test forward kinematics

% Function outputs
Ts = FK_space(M, screw_list_s, th_list);

Tb = fk_body(M, screw_list_b, th_list);

% Explicit calculations
Ts_test = eye(4);    % Ts = exp(S1*th1) * ... = exp(Sn*thn) * M
Tb_test = M;         % Tb = M * exp(S1*th1) * ... * exp(Sn*thn)
for i = 1:length(th_list)
    Ts_i = expm(screw2mat(screw_list_s(:, i)') * th_list(i));
    Tb_i = expm(screw2mat(screw_list_b(:, i)') * th_list(i));
    Ts_test = Ts_test * Ts_i;
    Tb_test = Tb_test * Tb_i;
end
Ts_test = Ts_test * M;

% Compare outputs
error_count = 0;
tol = 1e-4;

if ~all(ismembertol(Ts, Ts_test, tol), 'all')
    fprintf("Error: Spatial forward kinematics are wrong");
end

if ~all(ismembertol(Tb, Tb_test, tol), 'all')
    fprintf("Error: Body forward kinematics are wrong");
end

fprintf("Total errors: %d", error_count);