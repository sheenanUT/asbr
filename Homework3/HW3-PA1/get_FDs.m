function [FDs] = get_FDs(set_letter)
%GET_FDs Gets transforms FD from data files
%   Inputs:
%       set_letter = char, letter for the data set you need
%   Outputs:
%       FDs = 4 x 4 x Nframes matrix where each page is the transform from
%       d to D for that page     

% Get file names for current data set
% Datasets a - g are labeled "debug"
if ('a' <= set_letter) && (set_letter <= 'g')
    debug_label = 'debug';
% Data sets h - k are labeled "unknown"
else
    debug_label = 'unknown';
end
filename_body = ['pa1-', debug_label, '-', set_letter, '-calbody.txt'];
filename_readings = ['pa1-', debug_label, '-', set_letter, '-calreadings.txt'];

% Read data from files
[ds, ~, ~] = read_calbody(filename_body);
% No apparent use for measured C values
[Ds, ~, ~] = read_calreadings(filename_readings);

% Create empty array for FDs
N_frames = size(Ds, 3);
FDs = zeros(4, 4, N_frames);

% Get FD for each frame
for i = 1:N_frames
    FD_i = pc_reg(ds, Ds(:, :, i));
    FDs(:, :, i) = FD_i;
end

end