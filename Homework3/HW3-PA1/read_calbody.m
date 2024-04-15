function [ds, as, cs] = read_calbody(filename)
%READ_CALBODY This function reads the data on the calibration object from a
%"calbody" file
%   Inputs:
%       filename = string, name of calbody file to read
%   Outputs:
%     ds = 3xND matrix where each column is the position vector of an LED
%       marker on the EM base unit in the EM base unit frame
%     as = 3xNA matrix where each column is the position of an LED marker
%       on the calibration object in the calibration object frame
%     cs = 3xNC matrix where each column is the position of an EM marker on
%       the calibration object in the calibration object frame
end