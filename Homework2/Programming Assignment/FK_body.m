%% find the forward kinematics in the body frame
function T = FK_body(home_config, screw_list, theta_list)      % inputs are the home configurations, screw axes list, and theta list
    R_sb = home_config(1:3, 1:3);    % rotation matrix of b from s
    R_sb_t = transpose(R_sb);    % transpose of rotation matrix of b from s

    p_sb = home_config(1:3, 4);  % translation vector of b from s
    % translation vector of b from s under the se(3) form
    p_sb_skew = [
                0, -p_sb(3), p_sb(2);
                p_sb(3), 0, -p_sb(1);
                -p_sb(2), p_sb(1), 0];

    %adjacent matrix of inverse of home_config (M)
    adj_M_inv = [
    R_sb_t, zeros(3, 3);
    -R_sb_t*p_sb_skew, R_sb_t];


    % compute screw axis in the body frame and add to screw_b_list
    body_screw_list = [];       % initialize list for body screws in se(3) form
    for i = 1:length(screw_list)
        body_screw = adj_M_inv * screw_list(:, i);  % compute the screw in the body frame for S(i)
        body_screw_list = [body_screw_list, body_screw];    % add body screw axis to body_screw_list
    end

    % generate all body screw axes in se(3) form
    B_list = cell(1, length(screw_list));        % initilaize list for all body screws in se(3) form
    for i=1:length(screw_list)
        body_screw = body_screw_list(:, i);        % select body screw axis of column i
        w = body_screw(1:3);      % select first three elements of body screw axis to form omega vector
        v = body_screw(4:6);      % select last three elements of body screw axis to form velocity vector

        % form the skew symmetric matrix of omega
        w_skew = [
            0, -w(3), w(2);
            w(3), 0, -w(1);
            -w(2), w(1), 0];

        % form the body screw axis in se(3) form
        B = [
            w_skew, v;
            0, 0, 0, 0];
        B_list{i} = B;
    end

    
    % compute the body forward kinematics
    T = home_config;      % initialize the equation with home configuration
    for i = 1:length(B_list)
        B = B_list{i};      % use the current se(3) representation of the screw
        theta = theta_list(i);      % use current angle value

        exp_B_theta = expm(B*theta);    % exponential form of screw motion
        T = T * exp_B_theta;    % update the transformation matrix
    end
    
end