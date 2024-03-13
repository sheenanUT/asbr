function [Ts, S1, theta1] = screwaxis(T, q, s, h, theta)
%SCREWAXIS Calculates configurations of a rigid body traveling along a
%screw axis
%   Inputs:
%       T = 4x4 transformation matrix, initial configuration of body
%       q = 1x3 position vector, center of rotation
%       s = 1x3 vector, unit rotation axis
%       h = scalar, pitch, ratio of linear/angular velocity along/about
%       screw axis
%       theta = total angular distance to travel along screw axis, radians
%   Outputs:
%       Ts = 4x4x5 array of transformation matrices; Each element
%       represents the body's configuration at distance [0, theta/4,
%       theta/2, 3theta/4, theta] respectively
%       S1 = 1x6 vector, screw axis to return from final T to origin
%       theta1 = scalar distance along screw1 from final T to origin

% Initialize arrays
Ts = zeros(4, 4, 5);
thetas = [0, theta / 4, theta / 2, 3 * theta / 4, theta];

% Calculate normalized screw axis
w_norm = s; % Rotation vector = screw vector
v_norm = cross(-s, q) + h * s;
w_skew = v2skew(w_norm);
S = [w_norm, v_norm];

% Calculate orientation for each angle theta using matrix exponent
for i = 1:5
    T_screw = expm(screw2mat(S) * thetas(i));
'/
    % Apply screw transform to original orientation
    Ts(:, :, i) = T_screw * T;
end

% Plot each frame in 3D
hold on     % Composes all frames onto one plot
view(3);    % Ensures that plot is displayed in 3 dimensions
linespecs = ['r', 'g', 'b'];    % Color codes unit vectors (xyz = rgb)
axis equal; % Ensures that axes are scaled evenly

for i = 1:5
    % Plot from origin to endpoint of each unit vector
    origin = Ts(1:3, 4, i);
    R_shifted = Ts(1:3, 1:3, i) + origin; % Translates R to new origin

    for j = 1:3
        plot3([origin(1), R_shifted(1, j)],...
              [origin(2), R_shifted(2, j)],...
              [origin(3), R_shifted(3, j)], ...
              linespecs(j));
    end
end

% Calculate screw axis to return from final orientation to origin
% If I = T1 * T_final then T1 = I * T_final^-1
T1 = inv(Ts(:, :, 5));   % Transformation from final orientation to origin
R1 = T1(1:3, 1:3);      % Rotation matrix of T1
p1 = T1(1:3, 4);        % Translation vector of T1

% Matrix logarithm
if all(ismembertol(R1, eye(3), 1e-4))
    w1 = [0 0 0];       % Rotation vector
    v1 = p1 / norm(p1); % Translation vector
else
    axang = r2axisangle(R1);    % Get w1 and theta1 from R1
    w1 = axang(1:3);
    theta1 = axang(4);
    w1_skew = v2skew(w1);        % Skew-symmetric matrix of w1
    v1 = (1 / theta1 * eye(3) - 1 / 2 * w1_skew...    % G^-1 formula for v
        + (1 / theta1 - 1 / 2 * cot (theta1 / 2)) * w1_skew^2) * p1;
end
S1 = [w1 v1'];

% Validate outputs
% Show that e^[S1]theta1 * T_final = I
if ~all(ismembertol(eye(4), expm(screw2mat(S1) * theta1) * Ts(:,:,5), 1e-4))
    error("Could not verify that output S1 is the correct screw axis");
end

% Plot S1
origin1 = Ts(1:3, 4, 5);    % Origin of final orientation
w1_shifted = w1' + origin1; % w and v vectors shifted to body frame
v1_shifted = v1 + origin1;

plot3([origin1(1), w1_shifted(1)],...
      [origin1(2), w1_shifted(2)],...
      [origin1(3), w1_shifted(3)], ...
      '-^c',...
      [origin1(1), v1_shifted(1)],...
      [origin1(2), v1_shifted(2)],...
      [origin1(3), v1_shifted(3)], ...
      '->m');

hold off;
end
