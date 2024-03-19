%% find the forward kinematics in the body frame
function T = FK_body(home_config, body_screw_list, theta_list, body_q_list)     
%FK_BODY Calculates forward kinematics of robot in body frame
%   Inputs:
%       home_config = 4x4 transformation matrix representing zero-pose
%       configuration of robot end effector
%       body_screw_list = 6xn matrix where the columns are the robot's
%           screw axes in the body frame (n-joint robot)
%       theta_list = 1xn vector of joint angles for desired position
%       q_list = 3xn matrix where the columns are the positions of the
%           robot's joints in the zero-position, body-frame
%   Outputs:
%       T = 4x4 transformation matrix representing final configuration of
%           robot end effector in the body frame
%   Function also produces a 3D graph of the zero-pose frame, end-effector
%   frame, and joint screw axes

    % initialize list as cell array to separate each 4x4 matrix for all
    % body screws in se(3) form
    B_list = cell(1, length(body_screw_list));   

    % rotation vectors in homogeneous form
    w_list_h = zeros(4, length(body_screw_list));
    % position vectors in homogeneous form
    q_list_h = [body_q_list; ones(1, length(body_screw_list))];

    for i=1:length(body_screw_list)
        % select body screw axis of column i
        body_screw = body_screw_list(:, i);
        % get rotation vector from screw
        w = body_screw(1:3);      

        % body screw axis in se(3) form
        B = screw2mat(body_screw');

        B_list{i} = B;
        w_list_h(1:3, i) = w;
    end
    
    % compute the body forward kinematics
    T = home_config;      % initialize transform as home configuration
    for i = 1:length(B_list)
        B = B_list{i};      % use the current se(3) representation of the screw
        theta = theta_list(i);      % use current angle value

        exp_B_theta = expm(B*theta);    % exponential form of screw motion
        T = T * exp_B_theta;    % update the transformation matrix

        % transform w, q vectors to their new positions
        w_list_h(:, i) = T * w_list_h(:, i);
        q_list_h(:, i) = T * q_list_h(:, i);
    end
    
    % Plot frames and screw axes in 3D
    hold on;    % Plot all on 1 graph
    plot_config(T);     % End effector position
    plot_config(home_config);    % Body frame

    for i = 1:length(body_screw_list)
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