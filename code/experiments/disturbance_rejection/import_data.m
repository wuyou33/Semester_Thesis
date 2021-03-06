function [timestamp, az_reference, az_computed, az_measured] = importfile(filename, dataLines)
%IMPORTFILE Import data from a text file
%  [TIMESTAMP, AZ_REFERENCE, AZ_COMPUTED, AZ_MEASURED] =
%  IMPORTFILE(FILENAME) reads data from text file FILENAME for the
%  default selection.  Returns the data as column vectors.
%
%  [TIMESTAMP, AZ_REFERENCE, AZ_COMPUTED, AZ_MEASURED] =
%  IMPORTFILE(FILE, DATALINES) reads data for the specified row
%  interval(s) of text file FILENAME. Specify DATALINES as a positive
%  scalar integer or a N-by-2 array of positive scalar integers for
%  dis-contiguous row intervals.
%
%  Example:
%  [Timestamp, az_reference, az_computed, az_measured] = importfile("/home/evgheniivolodscoi/LRZ Sync+Share/TUM/Master_AS_RCI/Semester_3/Semesterarbeit/code/experiments/data/exp_1_acc.csv", [2, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 23-Mar-2020 22:36:42

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

%% Setup the Import Options
opts = delimitedTextImportOptions("NumVariables", 4);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Timestamp", "az_reference", "az_computed", "az_measured"];
opts.VariableTypes = ["double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
tbl = readtable(filename, opts);

%% Convert to output type
timestamp = tbl.Timestamp;
az_reference = tbl.az_reference;
az_computed = tbl.az_computed;
az_measured = tbl.az_measured;
end