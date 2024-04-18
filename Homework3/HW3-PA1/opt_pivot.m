function [t_g, P_dimple] = opt_pivot(filename_opt, filename_body)
    [Ds, Hopt] = read_optpivot(filename_opt);     % all frames of G data
    [ds, ~, ~] = read_calbody(filename_body);

    % convert H from optical frame to EM frame
    Hem = zeros(size(Hopt));
    for i = 1:size(Hopt, 3)
        invFD = pc_reg(Ds(:,:,i), ds);      % transformation matrix from optical frame to EM frame
        Hopt_framei = Hopt(:, :, i);        % current frame of H from optical frame

        Hem_framei = Hem(:, :, i);
        for k = 1:size(Hopt_framei, 1)
            Hopt_k_framei = Hopt_framei(k, :)';      % current position in current frame

            % perform the transform for current position in current frame
            Hem_k_framei = invFD(1:3, 1:3) * Hopt_k_framei + invFD(1:3, 4);
            Hem_framei(k, :) = Hem_k_framei;
        end
        Hem(:, :, i) = Hem_framei;
    end

    Hj_frame1 = Hem(:, :, 1);     % first frame of Hs
    Hs = Hem;
    
    % find sum of Hj_1
    sumHj_frame1 = zeros(1, size(Hj_frame1, 2));
    for i = 1:size(Hj_frame1, 1)
        Hi_frame1 = Hj_frame1(i, :);       % current iteration of Hj_frame(1)
        sumHj_frame1 = sumHj_frame1 + Hi_frame1;
    end
    
    H0 = (1/size(Hi_frame1, 1)) * sumHj_frame1;       % compute local probe

    % find gj for every frame
    gj = zeros(size(Hs, [1 2 3]));
    for i = 1:size(Hs, 3)
        % generate current frame of gj
        gj_framei=zeros(size(Hs, [1 2]));
        Hj_framei = Hs(:, :, i);       % current frame of Hj
        for j = 1:size(Hj_framei, 1)
            Hi_framei = Hj_framei(j, :);       % current iteration of Hj_frame(i)
            hi_framei = Hi_framei - H0;     % solve for current iteration of hj_frame(i)
            hj_framei(j, :) = hi_framei;
        end
        hj(:, :, i) = hj_framei;        % set current frame of hj
    end
    
    FH = zeros([4, 4, size(Hs, 3)]);
    for i = 1:size(Hs, 3)
        FH(:, :, i) = pc_reg(hj(:, :, 1), Hs(:, :, i));      % find all transforms and stack them only using frame 1 for gj
    end

    [t_g, P_dimple] = pivot_calibration(FH);
end