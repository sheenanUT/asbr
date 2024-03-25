function [thetas_d] = J_transpose_kinematics(home_config, screw_list, theta_list, q_list, T_d)
%J_TRANSPOSE_KINEMATICS Calculates inverse kinematics using Jacobian
%transpose method
%   Inputs:
%      home_config = 4x4 transformation matrix representing zero-pose
%       configuration of robot end effector
%       screw_list = 6xn matrix where the columns are the robot's screw
%           axes in the space frame (n-joint robot)
%       theta_list = 1xn vector of current joint angles
%       q_list = 3xn matrix where the columns are the positions of the
%           robot's joints in the zero-position, space-frame
%       T_d = 4x4 transformation matrix representing desired end effector
%       configuration
%   Outputs:
%       thetas_d = nx1 vector of joint angles that will move the end
%       effector to the desired position

% Get desired twist from T_d
[screw_d, theta_d] = t2screw(T_d);
twist_d = screw_d * theta_d;

% Set initial error to 1 so function always completes first loop
err = 1;

% Iterate until error is sufficiently low
tol = 1e-4;
while any(abs(err) > tol)
    if length(err) ~= 1   % Skip this section in first loop
        % Need a nxn symmetric positive definite matrix
        % Try identity times constant
        % Samuel Buss's formula, remember to cite
        alpha = dot(err, J_c * J_c' * err) /...
                dot(J_c * J_c' * err, J_c * J_c' * err);
        %alpha = 1;
        K = alpha * eye(length(theta_list));

        % Check Lyapunov condition
        if alpha <= 0
            error("Negative alpha");
        end

        % Estimate change in joint angles
        delta_theta = J_c' * K * err;
    
        % Displace current joint angles
        theta_list = theta_list + delta_theta';
    end

    % Compute current end effector configuration
    T_c = FK_space(home_config, screw_list, theta_list, q_list, false);
    
    % Get current twist from T_c
    [screw_c, theta_c] = t2screw(T_c);
    twist_c = screw_c * theta_c;

    % Compute current Jacobian
    J_c = J_space(screw_list, theta_list);

    % Compute error vector
    err = twist_d - twist_c;
end

thetas_d = theta_list;
end