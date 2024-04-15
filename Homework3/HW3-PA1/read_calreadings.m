function [Ds, As, Cs] = read_calreadings(filename)
%READ_CALBODY This function reads the data on the sensor readings for the 
%calibration object from a "calreadings" file
%   Inputs:
%       filename = string, name of calbody file to read
%   Outputs:
%      Ds = 3xNDxNframes matrix where each column is the position vector of an
%           LED marker on the EM base unit in optical tracker frame and each
%           page is a different frame of data
%      As = 3xNAxNframes matrix where each column is the position of an LED
%           marker on the calibration object in the optical tracker frame and
%           each page is a different frame of data
%      Cs = 3xNCxNframes matrix where each column is the position of an EM
%           marker on the calibration object in the EM base unit frame and each
%           page is a different frame of data
end