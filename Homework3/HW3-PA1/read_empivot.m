function [Gs] = read_empivot(filename)
%READ_EMPIVOT This function reads the data on the EM probe from a "empivot"
%file
%   Inputs:
%       filename = string, name of calbody file to read
%   Outputs:
%      Gs = 3xNGxNframes matrix where each column is the position vector of
%           an EM marker on the EM probe relative to the EM tracker and
%           each page is a different frame of data
end