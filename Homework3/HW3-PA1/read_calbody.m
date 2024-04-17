function [ds, as, cs] = read_calbody(filename)
%READ_CALBODY This function reads the data on the calibration object from a
%"calbody" file
%   Inputs:
%       filename = string, name of calbody file to read
%   Outputs:
%     ds = ND x 3 matrix where each row is the position vector of an LED
%       marker on the EM base unit in the EM base unit frame
%     as = NA x 3 matrix where each row is the position of an LED marker
%       on the calibration object in the calibration object frame
%     cs = NC x 3 matrix where each row is the position of an EM marker on
%       the calibration object in the calibration object frame

% Assume the input file has the correct format
data = importdata(filename);

% Get ND, NA, NC from header
header = data.textdata{1};  % First line of file as string
Ns_array = sscanf(header, "%d, %d, %d");    % Extract numbers
ND = Ns_array(1);
NA = Ns_array(2);
NC = Ns_array(3);

% Divide numerical data by type (d, a, c)
% Use N values to find dividing lines between data types
num_data = data.data;
ds = num_data(1 : ND, :);
as = num_data(ND + 1 : ND + NA, :);
cs = num_data(ND + NA + 1 : ND + NA + NC, :);
end