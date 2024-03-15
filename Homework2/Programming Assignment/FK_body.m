function T = fk_body(home_config, screw_list, theta_list)
%FK_BODY Calculates forward kinematics of robot in body frame
%   Inputs:
%       home_config = 4x4 transformation matrix representing the end
%           effector's zero-position
%       screw_list = 6xn matrix where each column is the screw axis for one
%           of the robot's joints (n joints total)
%       theta_list = 1xn vector of joint angles
%   Outputs:
%       T = transformation matrix representing end effector's position with
%       given joint angles

S_list = cell(1, length(screw_list));   % initilaize list for all screws in se(3) form
    T = home_config;      % add the home configuration into the equation

    % generate all screw axes in se(3) form
    for i=1:length(screw_list)
        screw = screw_list(:, i);        % select screw axis of column i

        % form the screw axis in se(3) form
        S = screw2mat(screw');

        S_list{i} = S;
    end

    % compute the forward kinematics
    for i = 1:length(screw_list)     % count up from 1 to n
        S = S_list{i};      % use the current se(3) representation of the screw
        theta = theta_list(i);      % use current angle value

        exp_S_theta = expm(S*theta);    % exponential form of screw motion
        T = T * exp_S_theta;    % update the transformation matrix
    end
end