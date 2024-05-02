verbose = true;     % Toggle function readouts
rand_pose = false;   % Toggle randomized poses

%% Robot-Specific Variables
% link dimensions in m
L1 = 500 * 10^-3;
L2 = 250 * 10^-3;
L3 = 770 * 10^-3;
L4 = 70 * 10^-3;
L5 = 780 * 10^-3;
L6 = 215 * 10^-3;

% joint angles in degrees
if rand_pose
    % Generate random angles within joint ranges
    th1 = rand() * 360 - 180;
    th2 = rand() * 190 - 145;
    th3 = rand() * 180 - 30;
    th4 = rand() * 700 - 350;
    th5 = rand() * 250 - 125;
    th6 = rand() * 700 - 350;

    % Generate second pose for IK
    th1b = rand() * 360 - 180;
    th2b = rand() * 190 - 145;
    th3b = rand() * 180 - 30;
    th4b = rand() * 700 - 350;
    th5b = rand() * 250 - 125;
    th6b = rand() * 700 - 350;
else
    % Generic starting pose
    th1 = 0;
    th2 = -120;
    th3 = 90;
    th4 = 0;
    th5 = -90;
    th6 = 0;

    % Generic second pose for IK
    th1b = 45;
    th2b = -90;
    th3b = 15;
    th4b = 30;
    th5b = -60;
    th6b = 45;

    % ensure proper joint angles
    if abs(th1) > 180
        disp('Angle 1 out of range. Proper range: ±180°'); end
    if th2<-145 || 45<th2
        disp('Angle 2 out of range. Proper range: -145° / 45°'); end
    if th3<-30 || 150<th3
        disp('Angle 3 out of range. Proper range: -30° / 150°'); end
    if abs(th4) > 350
        disp('Angle 4 out of range. Proper range: ±350°'); end
    if abs(th5) > 125
        disp('Angle 5 out of range. Proper range: ±125°'); end
    if abs(th6) > 350
        disp('Angle 6 out of range. Proper range: ±350°'); end
end

th_list = [th1 th2 th3 th4 th5 th6] * pi/180;   % Convert to radians
th_list_2 = [th1b th2b th3b th4b th5b th6b] * pi/180;

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
w_list = [w1, w2, w3, w4, w5, w6];      % list of omegas

% screw axis position vectors, space frame
q1 = [0; 0; -L1];
q2 = [L2; 0; 0];
q3 = [L2+L3; 0; 0];
q4 = [L2+L3; 0; -L4];
q5 = [L2+L3+L5; 0; -L4];
q6 = [L2+L3+L5+L6; 0; -L4];
q_list = [q1, q2, q3, q4, q5, q6];  % list of q's

% velocity vectors
v_list = zeros([3, length(w_list)]);    % list of velocity vectors
% compute cross products of w and q. Add them to v_list
for i = 1:length(w_list)
    v = cross(-w_list(:, i), q_list(:, i));  % compute the cross product
    v_list(:, i) = v;   % add velocity vector element to v_list
end

%% compute spatial screw list
screw_list = zeros([6, length(w_list)]);        % list of screws
% format the screw axes using w and v. Add them to screw_list
for i = 1:length(w_list)
    screw = [w_list(:, i ); v_list(:, i)];  % stack w(i) and v(i)
    screw_list(:, i) = screw;   % add screw axis to screw_list
end

%% compute body screw list and q's
% transform screw list to body frame
adj_M_inv = adj_transform(inv(M));      % adjacent of home configuration
body_screw_list = adj_M_inv * screw_list;       % transforms screw axis to body frame

% transform q's to body frame
q_list_h = [q_list; ones(1, length(q_list))];  % homogeneous form
body_q_list_h = inv(M) * q_list_h;  % T_bs * q_s = q_b
body_q_list = body_q_list_h(1:3, :);