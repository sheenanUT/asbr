function Js = J_space(screw_list, theta_list)
%J_SPACE Calculates the Jacobian matrix in the space frame for the current
%joint state
%   Inputs:
%       screw_list = 6xn matrix where the columns are the robot's
%           screw axes in the space frame (n-joint robot)
%       theta_list = 1xn vector of current joint positions
%   Outputs:
%       Js = 6xn space-frame Jacobian matrix for current position

    n = length(screw_list);       % number of joints
    
    % Validate inputs
    % screw_list must be 6xn
    if ~isequal(size(screw_list), [6 n])
        error("Input screw_list is not a 6xn matrix");
    % theta_list must be 1xn
    elseif ~isequal(size(theta_list), [1 n])
        error("Input theta_list is not a 1xn matrix");
    end

    Js = zeros([6, n]);        % initialize list for jacobian columns

    % generate all screw axes in se(3) form up to S_i
    % initilaize list as cell array to separate each 4x4 matrix for all
    % screws in se(3) form
    S_list = cell(1, length(screw_list));        
    for i=1:length(screw_list)
        screw = screw_list(:, i);        % select screw axis of column i
    
        % form the screw axis in se(3) form
        S = screw2mat(screw');
        S_list{i} = S;
    end


    % generate space jacobian matrix
    for i = 1:n     % i = Jacobian column
        % generate matrix of product of exponentials
        % = e^[S_1]*th_1 * e^[S_2]*th_2...*e^[S_n]*th_n]
        exp_prod = eye(4);
        for j = 1 : i - 1
            % Column i is transformed by joints from i to i-1
            S = S_list{j};        % set current screw axis of [S] form
            theta = theta_list(j);        % set current joint angle

            % exp_prod * e^[S_j]*th_j
            exp_prod = exp_prod * expm(S*theta);    
        end

        % compute adjacent matrix of current exp_prod
        adj_exp = adj_transform(exp_prod);
        % compute the space jacobian column
        Js_col = adj_exp * screw_list(:, i);    
        % Add column to jacobian matrix
        Js(:, i) = Js_col;
    end
end