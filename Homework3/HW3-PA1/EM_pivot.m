function EM_pivot(filename)
    Gs = read_empivot(filename);     % all frames of G data
    Gj_frame1 = Gs(:, :, 1);     % first frame of Gs

    % find sum of Gj_1
    sumGj_frame1 = zeros(1, size(Gj_frame1, 2));
    for i = 1:size(Gj_frame1, 1)
        Gi_frame1 = Gj_frame1(i, :);       % current iteration of Gj_frame(1)
        sumGj_frame1 = sumGj_frame1 + Gi_frame1;
    end
    
    G0 = (1/size(Gi_frame1, 1)) * sumGj_frame1;       % compute local probe

    % find gj for every frame
    gj = zeros(size(Gs))
    for i = 1:size(Gs, 3)
        % generate current frame of gj
        gj_framei=zeros(size(Gs, [1 2]));
        Gj_framei = Gs(:, :, i);       % current frame of Gj
        for j = 1:size(Gj_framei, 1)
            Gi_framei = Gj_framei(j, :);       % current iteration of Gj_frame(i)
            gi_framei = Gi_framei - G0;     % solve for current iteration of gj_frame(i)
            gj_framei(j, :) = gi_framei;
        end
        gj(:, :, i) = gj_framei;        % set current frame of gj
    end

    
    %% compute local probe
    % % find sum of Gj
    % sumGj = zeros(size(Gj, [1 2]));     % initialize as zero matrix
    % for i = 1:size(Gj, 3);
    %     Gi = Gj(:,:,i);     % current frame of G

    %     sumGj = sumGj + Gi;     % add current frame of G
    % end
    % G0 = (1/size(Gj, 3)) * sumGj;       % compute local probe

    % % translate observations relative to midpoint
    % gj = zeros(size(Gj, 1), 3, size(Gj, 3));
    % for i = 1:size(Gj, 3);
    %     Gi = Gj(:,:,i);     % current frame of G

    %     gj(:, :, i) = Gi-G0;
    % end
end