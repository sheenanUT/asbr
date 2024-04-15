function [Ds, Hs] = read_optpivot(filename)
%READ_OPTPIVOT This function reads the data on the LED probe from a
%"optpivot" file
%   Inputs:
%       filename = string, name of calbody file to read
%   Outputs:
%       Ds = 3xNDxNframes matrix where each column is the position vector of an
%           LED marker on the EM base unit in the optical tracker frame and each
%           page is a different frame of data
%       Hs = 3xNHxNframes matrix where each column is the position vector of an
%           LED marker on the optical probe in the optical tracker frame and each
%           page is a different frame of data
end