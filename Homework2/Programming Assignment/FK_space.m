%% find the forward kinematics in the space frame
function T = FK_space(home_config, screw_list, theta_list, q_list, plt)
%FK_SPACE Calculates forward kinematics of robot in space frame
%   Inputs:
%       home_config = 4x4 transformation matrix representing zero-pose
%       configuration of robot end effector
%       screw_list = 6xn matrix where the columns are the robot's screw
%           axes in the space frame (n-joint robot)
%       theta_list = 1xn vector of joint angles for desired position
%       q_list = 3xn matrix where the columns are the positions of the
%           robot's joints in the zero-position, space-frame
%       plt = Boolean, function generates 3D plot of robot if true
%   Outputs:
%       T = 4x4 transformation matrix representing final configuration of
%           robot end effector in the space frame
%   Function also produces a 3D graph of the zero-pose frame, end-effector
%   frame, and joint screw axes

    n = length(theta_list);

    % Validate inputs
    % home_config must be valid transform
    if ~is_transform(home_config)
        error("Input home_config is not a valid transformation matrix");
    % screw_list should be 6xn
    elseif ~isequal(size(screw_list), [6 n])
        error("Input screw_list is not a 6xn matrix");
    % theta_list should be 1xn
    elseif ~isequal(size(theta_list), [1 n])
        error("Input theta_list is not a 1xn vector");
    % q_list should be 3xn
    elseif ~isequal(size(q_list), [3 n])
        error("Input q_list is not a 3xn matrix");
    end

    % initialize list as cell array to separate each 4x4 matrix for all
    % screws in se(3) form
    S_list = cell(1, n);

    % rotation vectors in homogeneous form
    w_list_h = zeros(4, n);
    % position vectors in homogeneous form
    q_list_h = [q_list; ones(1, n)];

    for i=1:n
        screw = screw_list(:, i);   % select screw axis of column i
        w = screw(1:3);     % rotation vector of selected screw

        % form the screw axis in se(3) form
        S = screw2mat(screw');
        S_list{i} = S;
        w_list_h(1:3, i) = w;
    end

    % compute the spacial forward kinematics
    T = eye(4);      % initialize the equation a 4x4 identity matrix
    for i = 1:n
        S = S_list{i};      % use the current se(3) representation of the screw
        theta = theta_list(i);      % use current angle value

        exp_S_theta = expm(S*theta);    % exponential form of screw motion
        T = T * exp_S_theta;    % update the transformation matrix

        % transform w, q vectors to their new positions
        w_list_h(:, i) = T * w_list_h(:, i);
        q_list_h(:, i) = T * q_list_h(:, i);
    end
    % multiply the home configuration at the end of the formula 
    T = T * home_config;

    % Plot frames and screw axes in 3D
    if plt
        hold on;    % Plot all on 1 graph
        plot_config(T);     % End effector position
        plot_frame(eye(3), [0 0 0]);    % Space frame
    
        for i = 1:n
            % Plot screw axis rotation vectors
            plot_vector(w_list_h(1:3, i)' / 2, q_list_h(1:3, i)', '-m');
            % Plot arrow on positive end
            plot_vector([0 0 0], q_list_h(1:3, i)' + w_list_h(1:3, i)' / 2,...
                '-m^');
            if i > 1
                % Plot robot segments
                plot_vector(q_list_h(1:3, i)' - q_list_h(1:3, i - 1)',...
                            q_list_h(1:3, i - 1)', '-ko')
            end
        end
        hold off;
    end
end