%% find the forward kinematics in the space frame
function T = FK_space(home_config, screw_list, theta_list)      % inputs are the home configurations, screw axis list, and theta list
    S_list = cell(1, length(screw_list));        % initilaize list for all screws in se(3) form
    T = home_config;      % add the home configuration into the equation

    % generate all screw axes in se(3) form
    for i=1:length(screw_list)
        screw = screw_list(:, i);        % select screw axis of column i

        % form the screw axis in se(3) form
        S = screw2mat(screw');

        S_list{i} = S;
    end

    % compute the forward kinematics
    for i = length(screw_list):-1:1     % count down from n to 1
        S = S_list{i};      % use the current se(3) representation of the screw
        theta = theta_list(i);      % use current angle value

        exp_S_theta = expm(S*theta);    % exponential form of screw motion
        T = exp_S_theta * T;    % update the transformation matrix
    end
end