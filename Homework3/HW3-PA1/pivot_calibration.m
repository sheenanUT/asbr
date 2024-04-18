function [b_tip, b_post] = pivot_calibration(Fk)
% PIVOT_CALIBRATION finds the translation vectors b_tip and b_post
% from Fk
%     Input:
%         Fk: 4 x 4 x Nframes matrix where each page is the transform
%         from the tracker to the probe
%     Outputs:
%         b_tip: translation vector of the tip from the probe
%         b_post: translation vector of the pivot from the tracker

    Rk = [];
    pk = [];
    Ik = [];

    % find all Rk and Pk and generate Ik
    for i=1:size(Fk, 3)
        Fk_i = Fk(:, :, i);      % set current transformation
        Rk_i = Fk(1:3, 1:3);      % find rotation matrix of current Fk
        pk_i = Fk(1:3, 4);        % find translation vectro of current Fk

        Rk = [Rk; Rk_i];        % vertically concatenate
        pk = [pk; pk_i];        % vertically concatenate
        Ik = [Ik; eye(3)];  % generate i number of 3x3 identity matrices vertically
    end

    % least squares method to solve for b_tip and b_post
    A = [Rk, -Ik];
    B = -pk;
    [x, flag] = lsqr(A, B);
    b_tip = x(1:3);
    b_post = x(4:6);
end
