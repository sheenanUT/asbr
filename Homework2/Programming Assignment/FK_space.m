%% find the forward kinematics in the space frame
function T = FK_space(home_config, screw_list, theta_list)      % inputs are the home configurations, screw axis list, and theta list
    S_list = cell(1, length(screw_list));        % initilaize list for all screws in se(3) form
    T = home_config;      % add the home configuration into the equation

    % generate all screw axes in se(3) form
    for i=1:length(screw_list)
        screw = screw_list(:, i);        % select screw axis of column i
        w = screw(1:3);      % select first three elements of screw axis to form omega vector
        v = screw(4:6);      % select last three elements of screw axis to form velocity vector
        
        % form the skew symmetric matrix of omega
        w_skew = [
            0, -w(3), w(2);
            w(3), 0, -w(1);
            -w(2), w(1), 0];

        % form the screw axis in se(3) form
        S = [
            w_skew, v;
            0, 0, 0, 0];

        S_list{i} = S;
    end

    % compute the forward kinematics
    for i = 1:length(screw_list)
        S = S_list{i};      % use the current se(3) representation of the screw
        theta = theta_list(i);      % use current angle value

        exp_S_theta = expm(S*theta);    % exponential form of screw motion
        T = exp_S_theta * T;    % update the transformation matrix
    end
end