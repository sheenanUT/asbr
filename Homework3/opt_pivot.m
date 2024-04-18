function [t_g, P_dimple] = opt_pivot(filename_opt, filename_body)
    [Ds, Hopt] = read_optpivot(filename_opt);     % all frames of G data
    [ds, ~, ~] = read_calbody(filename_body);

    % convert H from optical frame to EM frame
    for i = 1:size(Hopt, 3)
        invFD = pc_reg(Ds(:,:,i), ds);      % transformation matrix from optical frame to EM frame
        Hopt_framei = Hopt(:, :, i);        % current frame of H from optical frame

        Hem_framei = 
        for k = 1:size(Hopt_framei, 1)
            Hopt_k_framei = Hopt_framei(k, :)';      % current position in current frame

            % perform the transform for current position in current frame
            Hem_k_framei = invFD(1:3, 1:3) * Hopt_k_framei + invFD(1:3, 4);
        end
    end

    % Gj_frame1 = Gs(:, :, 1);     % first frame of Gs

    % % find sum of Gj_1
    % sumGj_frame1 = zeros(1, size(Gj_frame1, 2));
    % for i = 1:size(Gj_frame1, 1)
    %     Gi_frame1 = Gj_frame1(i, :);       % current iteration of Gj_frame(1)
    %     sumGj_frame1 = sumGj_frame1 + Gi_frame1;
    % end
    
    % G0 = (1/size(Gi_frame1, 1)) * sumGj_frame1;       % compute local probe

    % % find gj for every frame
    % gj = zeros(size(Gs, [1 2 3]));
    % for i = 1:size(Gs, 3)
    %     % generate current frame of gj
    %     gj_framei=zeros(size(Gs, [1 2]));
    %     Gj_framei = Gs(:, :, i);       % current frame of Gj
    %     for j = 1:size(Gj_framei, 1)
    %         Gi_framei = Gj_framei(j, :);       % current iteration of Gj_frame(i)
    %         gi_framei = Gi_framei - G0;     % solve for current iteration of gj_frame(i)
    %         gj_framei(j, :) = gi_framei;
    %     end
    %     gj(:, :, i) = gj_framei;        % set current frame of gj
    % end
    
    % FG = zeros([4, 4, size(Gs, 3)]);
    % for i = 1:size(Gs, 3)
    %     FG(:, :, i) = pc_reg(Gs(:, :, i), gj(:, :, 1));      % find all transforms and stack them only using frame 1 for gj
    % end

    % [t_g, P_dimple] = pivot_calibration(FG);
end