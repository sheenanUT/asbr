function [Gs] = read_empivot(filename)
% READ_EMPIVOT This function reads the data on the EM probe from a "empivot"
% file
%   Inputs:
%       filename = string, name of empivot file to read
%   Outputs:
%      Gs = NG x 3 x Nframes matrix where each column is the position vector of
%           an EM marker on the EM probe relative to the EM tracker and
%           each page is a different frame of data

% Assume the input file has the correct format
data = importdata(filename);

% Get NG, Nframes from header
% This function needs a different format because the header has the same
% number of columns as the body
NG = str2double(data.textdata{1});
Nframes = str2double(data.textdata{2});

% Create output array
Gs = zeros(NG, 3, Nframes);

% Populate array
num_data = data.data;
for i = 1:Nframes
    frame_start = (i - 1) * NG;
    Gs(:, :, i) = num_data(frame_start + 1 : frame_start + NG, :);
end

end