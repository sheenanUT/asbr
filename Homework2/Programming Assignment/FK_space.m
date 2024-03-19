%% find the forward kinematics in the space frame
function T = FK_space(home_config, screw_list, theta_list, q_list)      % inputs are the home configurations, screw axes list, and theta list
    % generate all screw axes in se(3) form
    S_list = cell(1, length(screw_list));   % initilaize list for all screws in se(3) form
    w_list_h = zeros(4, length(screw_list));   % rotation vectors in homogeneous form
    q_list_h = [q_list; ones(1, length(screw_list))];   % position vectors in homogeneous form

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
        w_list_h(1:3, i) = w;
    end

    % compute the spacial forward kinematics
    T = eye(4);      % initialize the equation a 4x4 identity matrix
    for i = 1:length(screw_list)
        S = S_list{i};      % use the current se(3) representation of the screw
        theta = theta_list(i);      % use current angle value

        exp_S_theta = expm(S*theta);    % exponential form of screw motion
        T = T * exp_S_theta;    % update the transformation matrix

        % transform w, q vectors to their new positions
        w_list_h(:, i) = T * w_list_h(:, i);
        q_list_h(:, i) = T * q_list_h(:, i);

    end
    T = T * home_config;        % multiply the home configuration at the end of the formula 

    % Plot frames and screw axes in 3D
    hold on;    % Plot all on 1 graph
    plot_config(T);     % End effector position
    plot_frame(eye(3), [0 0 0]);    % Space frame

    for i = 1:length(screw_list)
        % Problem: These q-vectors are actually an arbitrary point along
        % the screw axis, not the true location of the joint

        % Solution: The true path from one joint to the next is the
        % shortest path (aka perpendicular) from the first joint to the
        % next screw axis
        
        %{
        % This section attempts to solve the problem described above,
        % but it doesn't work for screw axes along the robot's body.
        % Leave commented for now.
        if i > 1
            new_q = sym('q', [3 1], "real");
            % eq1 says the correct path is perpendicular to w
            eq1 = dot(new_q - q_list_h(1:3, i - 1), w_list_h(1:3, i)) == 0;
            % eq2 and eq3 say the corrected q lies on the line defined by
            % w and the arbitrary q
            eq2 = (new_q(1) - q_list_h(1, i)) == (new_q(2) - q_list_h(2, i));
            eq3 = (new_q(1) - q_list_h(1, i)) == (new_q(3) - q_list_h(3, i));
            [new_q_1, new_q_2, new_q_3] = solve([eq1 eq2 eq3]);
            q_list_h(1:3, i) = [new_q_1; new_q_2; new_q_3];
        end
        %}

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