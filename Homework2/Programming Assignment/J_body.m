%% find the jacobian matrix in the body frame
function Jb = J_body(body_screw_list, theta_list)        % find the jacobian matrix in {b} using the list of screw axes and joint angles
    n = length(body_screw_list);       % length of body screw list
    % initialize list for jacobian columns which make up body jacobian matrix
    Jb = zeros([6, n]);        
    
    % generate all body screw axes in se(3) form up to S_i
    % initialize list as cell array to separate each 4x4 matrix for all body screws in se(3) form
    B_list = cell(1, length(body_screw_list));
    for i=1:length(body_screw_list)
        % select body screw axis of column i
        body_screw = body_screw_list(:, i);        
        
        % form the body screw axis in se(3) form
        B = screw2mat(body_screw');
        B_list{i} = B;
    end

    % generate body jacobian matrix
    for i = n : -1 : 1      % iterate backwards from n to 1, i = Jacobian column
        % generate matrix of product of exponentials | e^[B_n]*th_n...*e^[B_i+1]*th_i+1]
        exp_prod = eye(4);
        for j = n : -1 : i + 1     % iterate backwards from n to i+1
            % Column i is transformed by joints from n to i+1
            B = B_list{j};        % set n-j screw axis of [S] form
            theta = theta_list(j);        % set n-j joint angle

            exp_prod = exp_prod * expm(-B*theta);   % exp_prod * e^[-B_j]*th_j
        end

        % compute adjacent matrix of current exp_prod
        adj_exp = adj_transform(exp_prod);
        % compute the body jacobian column
        Jb_col = adj_exp * body_screw_list(:, i);      
        % Add column to jacobian matrix
        Jb(:, i) = Jb_col;  
    end
end