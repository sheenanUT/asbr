function [thetas_d, thetas_d_array] = J_transpose_kinematics(home_config, ...
                      body_screw_list, theta_list, body_q_list, T_d)
%J_TRANSPOSE_KINEMATICS Calculates inverse kinematics using Jacobian
%transpose method
%   Inputs:
%       home_config = 4x4 transformation matrix representing zero-pose
%       configuration of robot end effector
%       body_screw_list = 6xn matrix where the columns are the robot's
%       screw axes in the body frame (n-joint robot)
%       theta_list = 1xn vector of current joint angles
%       body_q_list = 3xn matrix where the columns are the positions of the
%           robot's joints in the zero-position, body-frame
%       T_d = 4x4 transformation matrix representing desired end effector
%       configuration
%   Outputs:
%       thetas_d = nx1 vector of joint angles that will move the end
%       effector to the desired position
%       thetas_d_array = nxk array of joint angles for each iteration,
%       where k is the total number of iterations

n = length(theta_list);

% Validate inputs
% home_config and T_d must be transformation matrices
if ~is_transform(home_config)
    error("Input home_config is not a valid transformation matrix");
elseif ~is_transform(T_d)
    error("Input T_d is not a valid transformation matrix");
% body_screw_list must be 6xn
elseif ~isequal(size(body_screw_list), [6 n])
    error("Input body_screw_list is not a 6xn matrix");
% theta_list must be 1xn
elseif ~isequal(size(theta_list), [1 n])
    error("Input theta_list is not a 1xn vector");
% body_q_list must be 3xn
elseif ~isequal(size(body_q_list), [3 n])
    error("Input body_q_list is not a 3xn matrix");
end

% Set initial error to 1 for loop logic (see below)
err = 1;

% Iterate until error is sufficiently low
% Separate error measures for angular and linear velocities
errw = 1e-3;       % error value of angular velocity
errv = 1e-3;       % error value of velocity
err_cond = true;   % condition for loop to continue

% Maximum iterations
% Large number because this algorithm can be slow
i = 0;
i_max = 1e5;

% Save each iteration of thetas for graphing
thetas_d_array = theta_list;

while err_cond && i < i_max
    if length(err) ~= 1   % Skip this section in first loop
        % Samuel Buss's formula, remember to cite
        alpha = dot(err, J_c * J_c' * err) /...
                dot(J_c * J_c' * err, J_c * J_c' * err);

        % Estimate change in joint angles
        delta_theta = alpha * J_c' * err;
    
        % Displace current joint angles
        theta_list = theta_list + delta_theta';

        % Save new joint angles
        thetas_d_array = [thetas_d_array; theta_list];
    end

    % Compute current end effector configuration
    T_c = FK_body(home_config, body_screw_list, theta_list, body_q_list, false);

    % Compute current Jacobian
    J_c = J_body(body_screw_list, theta_list);

    % Compute error vector
    T_err = inv(T_c) * T_d;
    [se3, th] = SE3log(T_err);
    err = se3toR6(se3) * th;

    % Determine whether condition is met
    err_cond = norm(err(1:3)) > errw || norm(err(4:6)) > errv;

    i = i + 1;
end

thetas_d = theta_list;

if i == i_max
    fprintf("Error: Jacobian transpose failed to converge\n");
end

end