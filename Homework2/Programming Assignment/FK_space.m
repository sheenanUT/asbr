%% find the forward kinematics in the space frame
function T = FK_space(home_config, screw_list, theta_list, q_list)      % inputs are the home configurations, screw axes list, and theta list
    % generate all screw axes in se(3) form
    S_list = cell(1, length(screw_list));   % initilaize list for all screws in se(3) form
    w_list_h = zeros(4, length(screw_list));   % rotation vectors in homogeneous form
    q_list_h = [q_list; ones(1, length(screw_list))];   % position vectors in homogeneous form
    for i=1:length(screw_list)
        screw = screw_list(:, i);        % select screw axis of column i
        w = screw(1:3);     % rotation vector of selected screw

        % form the screw axis in se(3) form
        S = screw2mat(screw');

        S_list{i} = S;
        w_list_h(1:3, i) = w;
    end

    % compute the forward kinematics
    T = eye(4);      % initialize the equation a 4x4 identity matrix
    for i = 1:length(screw_list)
        S = S_list{i};      % use the current se(3) representation of the screw
        theta = theta_list(i);      % use current angle value

        exp_S_theta = expm(S*theta);    % exponential form of screw motion
        T = T * exp_S_theta;    % update the transformation matrix
        % iteratively transform w vectors to their new positions
        if i < length(screw_list)
            w_list_h(:, i+1) = T * w_list_h(:, i+1);
            q_list_h(:, i+1) = T * q_list_h(:, i+1);
        end

    end
    T = T * home_config;        % multiply the home configuration at the end of the formula 

    % Plot frames and screw axes in 3D
    hold on;    % Plot all on 1 graph
    plot_config(T);     % End effector position
    plot_frame(eye(3), [0 0 0]);    % Space frame

    for i = 1:length(screw_list)
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