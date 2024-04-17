function [Ds, Hs] = read_optpivot(filename)
%READ_OPTPIVOT This function reads the data on the LED probe from a
%"optpivot" file
%   Inputs:
%       filename = string, name of calbody file to read
%   Outputs:
%       Ds = ND x 3 x Nframes matrix where each column is the position vector of an
%           LED marker on the EM base unit in the optical tracker frame and each
%           page is a different frame of data
%       Hs = NH x 3 x Nframes matrix where each column is the position vector of an
%           LED marker on the optical probe in the optical tracker frame and each
%           page is a different frame of data

% Assume the input file has the correct format
data = importdata(filename);

% Get ND, NH Nframes from header
header = data.textdata{1};  % First line of file as string
Ns_array = sscanf(header, "%d, %d, %d, %d");    % Extract numbers
ND = Ns_array(1);
NH = Ns_array(2);
Nframes = Ns_array(3);

% Create outputs arrays
Ds = zeros(ND, 3, Nframes);
Hs = zeros(NH, 3, Nframes);

% Populate arrays
num_data = data.data;
for i = 1:Nframes
    frame_start = (i - 1) * (ND + NH);
    Ds(:, :, i) = num_data(frame_start + 1 : frame_start + ND, :);
    Hs(:, :, i) = num_data(frame_start + ND + 1 : frame_start + ND + NH, :);
end

end