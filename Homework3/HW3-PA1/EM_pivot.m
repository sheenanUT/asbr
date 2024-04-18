function EM_pivot(filename)
    Gj = read_empivot(filename)     % all frames of G data
    %% compute local probe
    % find sum of Gj
    sumGj = zeros(size(Gj, [1 2]));     % initialize as zero matrix
    for i = 1:size(Gj, 3);
        Gi = Gj(:,:,i);     % current frame of G

        sumGj = sumGj + Gi;     % add current frame of G
    end
    G0 = (1/size(Gj, 3)) * sumGj;       % compute local probe

    % translate observations relative to midpoint
    gj = zeros(size(Gj, 1), 3, size(Gj, 3));
    for i = 1:size(Gj, 3);
        Gi = Gj(:,:,i);     % current frame of G

        gj(:, :, i) = Gi-G0;
    end






end