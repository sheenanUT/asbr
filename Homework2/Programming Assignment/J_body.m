%% find the jacobian matrix in the body frame
function Jb = J_body(body_screw_list, theta_list)        % find the jacobian matrix in {b} using the list of screw axes and joint angles
    Jb = [];        % initialize list for jacobian columns which make up body jacboian matrix
    n = length(body_screw_list);       % length of body screw list

    % generate all body screw axes in se(3) form up to S_i
    B_list = cell(1, length(body_screw_list));        % initilaize list as cell array to separate each 4x4 matrix for all body screws in se(3) form
    for i=1:length(body_screw_list)

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


    % generate body jacboian matrix
    Jb = [Jb, body_screw_list(:, n)];     % set first body jacobian column as B_n
    for i = 1:(n-1)     % loop from 1 to n-1
        % generate matrix of product of exponentials | e^[B_n]*th_n...*e^[B_2]*th_2]
        exp_prod = expm(B_list{n}*theta_list(n));        % initialized as e^[B_n]*th_n
        for j = 1:i     % loop from 1 to current i iteration
            B = B_list{n-j};        % set n-j screw axis of [S] form
            theta = theta_list(n-j);        % set n-j joint angle

            exp_prod = exp_prod * expm(B*theta);        % exp_prod * e^[B_n-i]*th_n-i
        end


        R = exp_prod(1:3, 1:3);     % rotation matrix of current exp_prod
        p = exp_prod(1:3, 4);       % translation vector of current exp_prod

        % translation vector of b from s under the se(3) form
        p_skew = [
            0, -p(3), p(2);
            p(3), 0, -p(1);
            -p(2), p(1), 0];

        % compute adjacent matrix of current exp_prod
        adj_exp = [
            R, zeros(3, 3);
            -R*p_skew, R];
        
        Jb_col = adj_exp * body_screw_list(:, n-i);        % compute the body jacobian column
        Jb = [Jb, Jb_col];      % append the body jacobian column
    end
end