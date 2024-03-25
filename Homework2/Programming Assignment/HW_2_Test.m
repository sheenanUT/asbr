% Toggle function readouts
verbose = false;

%% Robot-Specific Variables
% link dimensions in m
L1 = 500 * 10^-3;
L2 = 250 * 10^-3;
L3 = 770 * 10^-3;
L4 = 70 * 10^-3;
L5 = 780 * 10^-3;
L6 = 215 * 10^-3;

% joint angles in degrees
th1 = 0;
th2 = -120;
th3 = 90;
th4 = 0;
th5 = -90;
th6 = 0;
th_list = [th1 th2 th3 th4 th5 th6] * pi/180;   % Convert to radians

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


%% Part a/b: Find the FK from spatial frame using FK_space.m
% calculate and display the spacial forward kinematics
figure(1);
axis equal;
T_sb = FK_space(M, screw_list, th_list, q_list);
if verbose
    fprintf("Space-frame forward kinematics:\n");
    disp(T_sb);
end

%% Part c: Find the FK from body frame using FK_body.m
figure(2);
T_bs = FK_body(M, body_screw_list, th_list, body_q_list);
if verbose
    fprintf("Body-frame forward kinematics:\n");
    disp(T_bs);
end

%% Part d: Find the space and body form Jacobian of the robot
J_s = J_space(screw_list, th_list);       % find space jacobian
if verbose
    fprintf("Space-frame Jacobian:\n");
    disp(J_s);
end

J_b = J_body(body_screw_list, th_list);   % find body jacobian
if verbose
    fprintf("Body-frame Jacobian:\n");
    disp(J_b);
end

%% determine if robot is in a singularity configuration
if verbose
    singularity(J_s);       % display if robot is in singularity
end



%% Part h: Inverse Kinematics Function
% control the robot from arbitrary configuration a to b
T_desired = [
    -0.4924    0.8660    0.0868    0.4696
    0.1736   -0.0000    0.9848    0.0373
    0.8529    0.5000   -0.1504    1.1796
    0         0         0    1.0000];

theta_list_desired = J_inverse_kinematics(M, body_screw_list, th_list, body_q_list, T_desired);       % calculate angles to for desired end-effector position
disp(IKinBody(body_screw_list, M, T_desired, 0.01, 0.001));

%% Part g: Find and plot manipulability ellipsoids
% TODO: maybe write a function separate from FK_space that plots the robot
% TODO: figure out why J_s and J_b give different ellipsoids
% Linear manipulability
figure(3);
FK_space(M, screw_list, th_list, q_list);
ellipsoid_plot_linear(J_b, T_sb);   % Textbook says use body Jacobian

% Angular manipulability
figure(4);
FK_space(M, screw_list, th_list, q_list);
ellipsoid_plot_angular(J_b, T_sb);

% Isotropy
iso = J_isotropy(J_b);
if verbose
    fprintf("Isotropy = %d\n", iso);
end

% Condition
cond = J_condition(J_b);
if verbose
    fprintf("Condition = %d\n", cond);
end

% Volume
vol = J_volume(J_b);
if verbose
    fprintf("Volume = %d\n", vol);
end

% % Compare outputs
error_count = 0;
tol = 1e-4;


Ts_test = FKinSpace(M, screw_list, th_list');
Tb_test = FKinBody(M, body_screw_list, th_list');
% Compare outputs
if ~all(ismembertol(T_sb, Ts_test, tol), 'all')
    fprintf("Error: Spatial forward kinematics are wrong\n");
    error_count = error_count + 1;
end

if ~all(ismembertol(T_bs, Tb_test, tol), 'all')
    fprintf("Error: Body forward kinematics are wrong\n");
    error_count = error_count + 1;
end

%% Test Jacobian Functions
J_s_test = JacobianSpace(screw_list, th_list);
J_b_test = JacobianBody(body_screw_list, th_list);

% Compare outputs
if ~all(ismembertol(J_s, J_s_test, tol), 'all')
    fprintf("Error: Space Jacobian is wrong\n");
    error_count = error_count + 1;
end
if ~all(ismembertol(J_b, J_b_test, tol), 'all')
    fprintf("Error: Body Jacobian is wrong\n");
    error_count = error_count + 1;
end

% Test space-body relation
% J_b = Adj(T_bs) * J_s
if ~all(ismembertol(J_b, adj_transform(inv(T_sb)) * J_s))
    fprintf("Error: Space-Body relation test failed\n");
    error_count = error_count + 1;
end

%% Display test results
fprintf("Total errors: %d\n", error_count);

hold off;
