%% find the jacobian matrix in the space frame
function Js = J_space(screw_list, theta_list)     % find the jacobian matrix in {s} using the list of screw axes and list of joint angles
    n = length(screw_list);        % length of screw list
    Js = zeros([6, n]);        % initialize list for jacobian columns which make up space jacobian matrix

    % generate all screw axes in se(3) form up to S_i
    S_list = cell(1, length(screw_list));        % initilaize list as cell array to separate each 4x4 matrix for all screws in se(3) form
    for i=1:length(screw_list)
        screw = screw_list(:, i);        % select screw axis of column i
    
        % form the screw axis in se(3) form
        S = screw2mat(screw');
        S_list{i} = S;
    end


    % generate space jacobian matrix
    for i = 1:n     % i = Jacobian column
        % generate matrix of product of exponentials | e^[S_1]*th_1 * e^[S_2]*th_2...*e^[S_n]*th_n]
        exp_prod = eye(4);
        for j = 1 : i - 1
            % Column i is transformed by joints from i to i-1
            S = S_list{j};        % set current screw axis of [S] form
            theta = theta_list(j);        % set current joint angle

            exp_prod = exp_prod * expm(S*theta);    % exp_prod * e^[S_j]*th_j
        end

        % compute adjacent matrix of current exp_prod
        adj_exp = adj_transform(exp_prod);
        % compute the space jacobian column
        Js_col = adj_exp * screw_list(:, i);    
        % Add column to jacobian matrix
        Js(:, i) = Js_col;
    end
end