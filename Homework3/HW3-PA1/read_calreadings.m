function [Ds, As, Cs] = read_calreadings(filename)
%READ_CALBODY This function reads the data on the sensor readings for the 
%calibration object from a "calreadings" file
%   Inputs:
%       filename = string, name of calbody file to read
%   Outputs:
%      Ds = ND x 3 x Nframes matrix where each column is the position vector of an
%           LED marker on the EM base unit in optical tracker frame and each
%           page is a different frame of data
%      As = NA x 3 x Nframes matrix where each column is the position of an LED
%           marker on the calibration object in the optical tracker frame and
%           each page is a different frame of data
%      Cs = NC x 3 x Nframes matrix where each column is the position of an EM
%           marker on the calibration object in the EM base unit frame and each
%           page is a different frame of data

% Assume the input file has the correct format
data = importdata(filename);

% Get ND, NA, NC, Nframes from header
header = data.textdata{1};  % First line of file as string
Ns_array = sscanf(header, "%d, %d, %d, %d");    % Extract numbers
ND = Ns_array(1);
NA = Ns_array(2);
NC = Ns_array(3);
Nframes = Ns_array(4);

% Create outputs arrays
Ds = zeros(ND, 3, Nframes);
As = zeros(NA, 3, Nframes);
Cs = zeros(NC, 3, Nframes);

% Populate arrays
num_data = data.data;
for i = 1:Nframes
    frame_start = (i - 1) * (ND + NA + NC);
    Ds(:, :, i) = num_data(frame_start + 1 : frame_start + ND, :);
    As(:, :, i) = num_data(frame_start + ND + 1 : frame_start + ND + NA, :);
    Cs(:, :, i) = num_data(frame_start + ND + NA + 1 : frame_start + ND + NA + NC, :);
end

end