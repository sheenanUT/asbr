%% find the jacobian matrix in the space frame
function Js = J_space(screw_list, theta_list)     % find the jacobian matrix in {s} using the list of screw axes and list of joint angles
    Js = [];        % initialize list for jacobian columns which make up jacboian matrix

    % generate all screw axes in se(3) form up to S_i
    S_list = cell(1, length(screw_list));        % initilaize list as cell array to separate each 4x4 matrix for all screws in se(3) form
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

    % generate jacboian matrix
    Js = [Js, screw_list(:, 1)];     % set first jacobian
    for i = 1:(length(screw_list)-1)     % S_i | loop from 1 to i-1
        % generate matrix of product of exponentials | [e^S_1*th_1][e^S_2*th_2]...[e^S_i*th_i]
        exp_prod = eye(length(S_list(:, 1)));     % initialized as identity matrix
        for j = 1:i     % loop from 1 to current i iteration
            S = S_list{j};        % set current screw axis of [S] form
            theta = theta_list(j);        % set current joint angle

            exp_prod = exp_prod * expm(S*theta);        % exp_prod * e^[S_i]*th_i
        end

        R = exp_prod(1:3, 1:3);     % rotation matrix of current exp_prod
        p = exp_prod(1:3, 4);       % translation vector of current exp_prod

        % translation vector of b from s under the se(3) form
        p_skew = [
            0, -p(3), p(2);
            p(3), 0, -p(1);
            -p(2), p(1), 0];

        % compute adjacent matrix of current exp_prod
        adj_exp_i = [
            R, zeros(3, 3);
            -R*p_skew, R];

        Js_col = adj_exp_i * screw_list(:, i);        % compute the jacobian column
        Js = [Js, Js_col];      % append the jacobian column
    end
end