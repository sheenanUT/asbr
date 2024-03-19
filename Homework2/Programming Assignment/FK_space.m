%% find the forward kinematics in the space frame
function T = FK_space(home_config, screw_list, theta_list)      % inputs are the home configurations, screw axes list, and theta list
    % generate all screw axes in se(3) form
    S_list = cell(1, length(screw_list));        % initilaize list as cell array to separate each 4x4 matrix for all screws in se(3) form
    for i=1:length(screw_list)
        screw = screw_list(:, i);        % select screw axis of column i

        % form the screw axis in se(3) form
        S = screw2mat(screw');

        S_list{i} = S;
    end

    % compute the forward kinematics
    T = eye(4);      % initialize the equation a 4x4 identity matrix
    for i = 1:length(screw_list)
        S = S_list{i};      % use the current se(3) representation of the screw
        theta = theta_list(i);      % use current angle value

        exp_S_theta = expm(S*theta);    % exponential form of screw motion
        T = T * exp_S_theta;    % update the transformation matrix
    end
    T = T * home_config;        % multiply the home configuration at the end of the formula 

end