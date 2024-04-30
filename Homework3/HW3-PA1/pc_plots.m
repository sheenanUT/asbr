% Script to test plotting point clouds

i = 'g';

% Datasets a - g are labeled "debug"
if ('a' <= i) && (i <= 'g')
    debug_label = 'debug';
% Data sets h - k are labeled "unknown"
else
    debug_label = 'unknown';
end
filename_body = ['pa1-', debug_label, '-', i, '-calbody.txt'];
filename_readings = ['pa1-', debug_label, '-', i, '-calreadings.txt'];

% Read data from files
[ds, as, cs] = read_calbody(filename_body);
% No apparent use for measured C values
[Ds, As, Cs_measured] = read_calreadings(filename_readings);

ND = size(ds, 1);
NA = size(as, 1);
NC = size(cs, 1);

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
    Fc = inv(Fd) * Fa;

    % Calculate expected C values
    Cs_homo_j = Fc * cs_homo;
    Cs_j = Cs_homo_j(1:3, :)';
    Cs(:, :, j) = Cs_j;
end

filename_output = ['pa1-debug-', i, '-output1.txt'];
Cs_debug = read_output(filename_output);

Cs_1 = Cs;
Cs_2 = Cs_debug;
frame = 3;
    
err = abs(Cs_1(:, :, frame) - Cs_2(:, :, frame));
fprintf("Dataset " + i + ", Frame " + frame + ":\n" + ...
    "Average error = " + string(mean(err, "all")) + "\n" + ...
    "Max error = " + string(max(err, [], "all")) + "\n");

% Plot vector from each
clf;
hold on;
for i = 1:NC
    plot_vector(Cs_1(i, :, frame) - Cs_2(i, :, frame), Cs_2(i, :, frame), '-k');
    plot_vector([0 0 0], Cs_1(i, :, frame), '-bo');
    plot_vector([0 0 0], Cs_2(i, :, frame), '-ro');
end
hold off;
