% variables %
% symbols
syms L      % length
syms th1 th2 th3 th4      % angles [theta]

% thetas
th1 = 0;
th2 = 0;
th3 = pi/2;
th4 = L;

% axis of rotation vectors
w1 = [0; 0; 1];
w2 = [1; 0; 0];
w3 = [0; 0; 1];
w4 = [0; 0; 0];

% distance vectors
% distance from spacial origin to screw axes
q1 = [0; 0; 0];
q2 = [0; 0; L];
q3 = [-L*cos(th2)*sin(th1); L*cos(th2)*cos(th1); L+L*cos(th1)*sin(th2)];
% distance from spacial origin to body origin
p_sb = [-L*cos(th2)*sin(th1) - th4*cos(th2)*sin(th1+th3);
    L*cos(th1)*cos(th2) + th4*cos(th1+th3)*cos(th2);
    L + L*cos(th1)*sin(th2) + th4*cos(th1+th3)*sin(th2)];

% rotation matrices
R_sb = [
    1, 0, 0;
    0, 1, 0;
    0, 0, 1;];

R_sb_t = transpose(R_sb);

% distance matrices
p_sb_scew = [
    0, -p_sb(3), p_sb(2);
    p_sb(3), 0, -p_sb(1);
    -p_sb(2), p_sb(1), 0];

% adjacent matrices
adjacent_matrix_sb = [
    R_sb_t, zeros(3);
    -R_sb_t*p_sb_scew, R_sb_t];


% main %
% velocity vectors
v1 = cross(q1, w1);
v2 = cross(q2, w2);
v3 = cross(q3, w3);
v4 = [
    -cos(th2)*sin(th1+th3);
    cos(th2)*cos(th1+th3);
    sin(th2)*cos(th1+th3)];

% jacobian matrices 
J_s = [
    w1, w2, w3, w4;
    v1, v2, v3, v4;];

J_b = adjacent_matrix_sb * J_s;
disp(vpa(J_b))

tip_vel = J_b * [1;1;1;1];
disp(vpa(tip_vel))

