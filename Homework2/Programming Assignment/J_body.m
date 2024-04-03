function Jb = J_body(body_screw_list, theta_list)
%J_BODY Calculates the Jacobian matrix in the body frame for the current
%joint state
%   Inputs:
%       body_screw_list = 6xn matrix where the columns are the robot's
%           screw axes in the body frame (n-joint robot)
%       theta_list = 1xn vector of current joint positions
%   Outputs:
%       Jb = 6xn body-frame Jacobian matrix for current position

    n = length(body_screw_list);       % number of joints
    
    % Validate inputs
    % body_screw_list must be 6xn
    if ~isequal(size(body_screw_list), [6 n])
        error("Input body_screw_list is not a 6xn matrix");
    % theta_list must be 1xn
    elseif ~isequal(size(theta_list), [1 n])
        error("Input theta_list is not a 1xn matrix");
    end

    
    % initialize list for jacobian columns
    Jb = zeros([6, n]);        
    
    % generate all body screw axes in se(3) form up to S_i
    % initialize list as cell array to separate each 4x4 matrix for all
    % body screws in se(3) form
    B_list = cell(1, length(body_screw_list));
    for i=1:length(body_screw_list)
        % select body screw axis of column i
        body_screw = body_screw_list(:, i);        
        
        % form the body screw axis in se(3) form
        B = screw2mat(body_screw');
        B_list{i} = B;
    end

    % generate body jacobian matrix
    % i = Jacobian column
    for i = n : -1 : 1      % iterate backwards from n to 1
        % generate matrix of product of exponentials
        % = e^[B_n]*th_n...*e^[B_i+1]*th_i+1]
        exp_prod = eye(4);
        for j = n : -1 : i + 1     % iterate backwards from n to i+1
            % Column i is transformed by joints from n to i+1
            B = B_list{j};        % set n-j screw axis of [S] form
            theta = theta_list(j);        % set n-j joint angle

            % exp_prod * e^[-B_j]*th_j
            exp_prod = exp_prod * expm(-B*theta);   
        end

        % compute adjacent matrix of current exp_prod
        adj_exp = adj_transform(exp_prod);
        % compute the body jacobian column
        Jb_col = adj_exp * body_screw_list(:, i);      
        % Add column to jacobian matrix
        Jb(:, i) = Jb_col;  
    end
end