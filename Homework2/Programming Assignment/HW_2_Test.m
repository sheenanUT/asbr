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
th1 = 0;
th2 = -120;
th3 = 60;
th4 = 0;
th5 = 0;
th6 = 0;
th_list = [th1 th2 th3 th4 th5 th6] * pi/180;   % Convert to radians

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
w_list = [w1, w2, w3, w4, w5, w6];  % list of omegas

% Screw axis position vectors, space frame
% Problem: This only works when the q's are at the exact locations of the
% joints. For now the arbitrary q's are commented out in favor of the
% precise q's.
qs1 = [0; 0; -L1];
qs2 = [L2; 0; 0];
qs3 = [L2+L3; 0; 0];
qs4 = [L2+L3; 0; -L4];
%qs4 = [0; 0; -L4];
qs5 = [L2+L3+L5; 0; -L4];
%qs5 = [L2+L3+L5; 0; 0];
qs6 = [L2+L3+L5+L6; 0; -L4];
%qs6 = [0; 0; -L4];
q_list = [qs1, qs2, qs3, qs4, qs5, qs6];  % list of q's


% Screw axis position vectors, body frame
% TODO: determine whether these are still necessary
qb1 = [-L6-L5-L3-L2; 0; 0];
qb2 = [-L6-L5-L3; 0; L4];
qb3 = [-L6-L5; 0; L4];
qb4 = [-L6; 0; 0];
qb5 = [-L6; 0; 0];
qb6 = [0; 0; 0];
qb_list = [qb1, qb2, qb3, qb4, qb5, qb6];

v_list = [];    % list of velocity vectors
% compute cross products of w and q. Add them to v_list
for i = 1:length(w_list)
    v = cross(w_list(:, i), q_list(:, i));  % compute the cross product
    v_list = [v_list, v];   % add velocity vector element to v_list
end

%% compute spatial screw list
screw_list = [];        % list of screws
% format the screw axes using w and v. Add them to screw_list
for i = 1:length(w_list)
    screw = [w_list(:, i ); v_list(:, i)];  % stack w(i) and v(i)
    screw_list = [screw_list, screw];   % add screw axis to screw_list
end



%% compute body screw list
R_sb = M(1:3, 1:3);    % rotation matrix of b from s
R_sb_t = transpose(R_sb);    % transpose of rotation matrix of b from s

p_sb = M(1:3, 4);  % translation vector of b from s
% translation vector of b from s under the se(3) form
p_sb_skew = v2skew(p_sb')

% adjacent matrix of inverse of home_config (M)
adj_M_inv = [
R_sb_t, zeros(3, 3);
-R_sb_t*p_sb_skew, R_sb_t];


% compute screw axis in the body frame and add to screw_b_list
body_screw_list = [];       % initialize list for body screws
for i = 1:length(screw_list)
    body_screw = adj_M_inv * screw_list(:, i);  % compute the screw in the body frame for S_i
    body_screw_list = [body_screw_list, body_screw];    % add body screw axis to body_screw_list
end



%% Part a: Find the FK from spatial frame using FK_space.m
% calculate and display the spacial forward kinematics
T_sb = FK_space(M, screw_list, th_list, q_list);        % find FK in {s}
disp(T_sb);



%% Part c: Find the forward kinematics in reference to the body frame | FK_body Function
T_bs = FK_body(M, body_screw_list, th_list);     % find FK in {b}
disp(T_bs);



%% Part d: Find the space and body form Jacobian of the robot
J_s = J_space(screw_list, th_list);       % find space jacobian
disp(J_s);

J_b = J_body(body_screw_list, th_list);        % find body jacobian
disp(J_b);



%% Test Forward Kinematics

% Explicit calculations
Ts_test = eye(4);    % Ts = exp(S1*th1) * ... = exp(Sn*thn) * M
Tb_test = M;         % Tb = M * exp(S1*th1) * ... * exp(Sn*thn)
for i = 1:length(th_list)
    Ts_i = expm(screw2mat(screw_list(:, i)') * th_list(i));
    Tb_i = expm(screw2mat(body_screw_list(:, i)') * th_list(i));
    Ts_test = Ts_test * Ts_i;
    Tb_test = Tb_test * Tb_i;
end
Ts_test = Ts_test * M;

% Compare outputs
error_count = 0;
tol = 1e-4;

if ~all(ismembertol(T_sb, Ts_test, tol), 'all')
    fprintf("Error: Spatial forward kinematics are wrong\n");
    error_count = error_count + 1;
end

if ~all(ismembertol(T_bs, Tb_test, tol), 'all')
    fprintf("Error: Body forward kinematics are wrong\n");
    error_count = error_count + 1;
end

fprintf("Total errors: %d", error_count);
