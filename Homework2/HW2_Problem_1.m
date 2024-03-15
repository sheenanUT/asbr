syms L1
syms L2
syms L3
syms L4
syms L5
syms L6

W = [
    0;
    1;
    0];

Q = [
    L1;
    L3+L4;
    -L5];

V = cross(Q,W);
% V = [0;1;0]

S = [
    W(1);
    W(2);
    W(3);
    V(1);
    V(2);
    V(3)]

R = [
    1, 0, 0;
    0, 1, 0;
    0, 0, 1];
R_t = transpose(R);

p = [L1; L3+L4; -L5-L6];
p_scew = [
    0, -p(3), p(2);
    p(3), 0, -p(1);
    -p(2), p(1), 0];

adjacent_matrix = [
    R_t, zeros(3);
    -R_t*p_scew, R_t];

B = adjacent_matrix * S

% M = [
%     1, 0, 0, L1;
%     0, 1, 0, L3+L4;
%     0, 0, 1, -L5-L6;
%     0, 0, 0, 1];