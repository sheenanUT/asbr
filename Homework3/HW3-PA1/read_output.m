function [Cs, post_EM, post_opt] = read_output(filename)
%READ_OUTPUT This function reads the expected C values from an output file
%   Inputs:
%       filename = string, name of output file to read
%   Outputs:
%       Cs = NC x 3 x Nframes matrix where each row is the expected position of an EM
%            marker on the calibration object in the EM base unit frame and each
%            page is a different frame of data
%       post_EM = 1 x 3 vector representing expected position of EM post
%       post_Opt = 1 x 3 vector representing expected position of optical
%                  post

% Assume the input file has the correct format
data = importdata(filename);

% Get Nc, Nframes from header
NC = str2double(data.textdata{1});
Nframes = str2double(data.textdata{2});

% Create output array
Cs = zeros(NC, 3, Nframes);

% Populate array
num_data = data.data;
for i = 1:Nframes
    % Skip first 2 lines
    frame_start = (i - 1) * NC + 2;
    Cs(:, :, i) = num_data(frame_start + 1 : frame_start + NC, :);
end

% Get post data
post_EM = num_data(1, :);
post_opt = num_data(2, :);

end