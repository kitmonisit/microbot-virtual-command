% HGTable - Functions to display tabular data in a MATLAB list control
%
% A standard MATLAB listbox is used to display tabular data.  "Columns" of
% data are supplied as cell arrays of strings, and a column name is given for each.
% The individual strings are padded with spaces to make them all the same
% width, and the columns are joined together to produce a table.
%
% Functions for normal usage:
%   tablesetup - Sets up a listbox for use with tablular data
%   formattable - Formats header and column data and displays it in the listbox
%
% Additional methods:
%   tablecolumn - Formats a single column of data for use in the table
%   tablestrings - Combines multiple columns into a single

