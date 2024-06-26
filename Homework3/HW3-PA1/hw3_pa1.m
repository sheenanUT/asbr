% Homework 3
% Programming Assignment 1
% Part 3: Calibration

% This script uses the data in the calbody and calreadings files to compute
% the expected values of Ci, the positions of the EM markers on the
% calibration object with respect to the EM tracker. This calculation is
% performed for each frame of each given data set. The values are then
% output into a series of .txt files.

% Data sets are given letters from 'a' to 'k'
for i = 'a':'k'
    % Get file names for current data set
    % Datasets a - g are labeled "debug"
    if ('a' <= i) && (i <= 'g')
        debug_label = 'debug';
    % Data sets h - k are labeled "unknown"
    else
        debug_label = 'unknown';
    end
    filename_body = "pa1-" + debug_label + "-" + i + "-calbody.txt";
    filename_readings = "pa1-" + debug_label + "-" + i + "-calreadings.txt";
    filename_empivot = "pa1-" + debug_label + "-" + i + "-empivot.txt";
    filename_optpivot = "pa1-" + debug_label + "-" + i + "-optpivot.txt";

    % Read data from files
    [ds, as, cs] = read_calbody(filename_body);
    % No apparent use for measured C values
    [Ds, As, ~] = read_calreadings(filename_readings);

    % Create empty array for Cs
    N_frames = size(As, 3);
    Nc = size(cs, 1);
    Cs = zeros(Nc, 3, N_frames);

    % Homogeneous form for cs
    cs_homo = [cs'; ones(1, Nc)];

    % Iterate over each data frame
    for j = 1:N_frames
        % Get transforms Fd and Fa
        Fd = pc_reg(ds, Ds(:, :, j));
        Fa = pc_reg(as, As(:, :, j));

        % Calculate Fc = inv(Fd) * Fa
        Fc = Fd \ Fa;

        % Calculate expected C values
        Cs_homo_j = Fc * cs_homo;
        Cs_j = Cs_homo_j(1:3, :)';
        Cs(:, :, j) = Cs_j;
    end

    % Pivot calibration for EM probe
    [~, P_dimple_EM] = EM_pivot(filename_empivot);

    % Pivot calibration for optical probe
    [~, P_dimple_opt] = opt_pivot(filename_optpivot, filename_body);

    % Check with debug data if available
    if isequal(debug_label, 'debug')
        filename_output = ['pa1-debug-', i, '-output1.txt'];
        [Cs_debug, EM_debug, opt_debug] = read_output(filename_output);
        Cs_err = abs(Cs - Cs_debug);
        EM_err = abs(P_dimple_EM - EM_debug);
        opt_err = abs(P_dimple_opt - opt_debug);
        fprintf("Dataset " + i + ":\n" + ...
            "Calibration error = " + string(mean(Cs_err, "all")) + "\n" + ...
            "EM pivot error = " + string(mean(EM_err, "all")) + "\n" + ...
            "Optical pivot error = " + string(mean(opt_err, "all")) + "\n" + ...
            "\n");
    end

    % Write output to file
    folder_name = "Homework3/outputs/"; % Change this if needed
    filename_my_output = "pa1-" + debug_label + "-" + i + "-my-output1.txt";
    fileID = fopen(folder_name + filename_my_output, 'w');
    fprintf(fileID, string(Nc) + ", " + string(N_frames) + ", " + filename_my_output + "\n");

    % Write pivot calibration results
    fprintf(fileID, "%8.2f, %8.2f, %8.2f\n", P_dimple_EM);
    fprintf(fileID, "%8.2f, %8.2f, %8.2f\n", P_dimple_opt);

    % Write Cs to file
    for j = 1:N_frames
        for k = 1:Nc
            fprintf(fileID, "%8.2f, %8.2f, %8.2f\n", Cs(k, :, j));
        end
    end
    fclose(fileID);

end