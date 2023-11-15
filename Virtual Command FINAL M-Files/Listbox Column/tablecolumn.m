function column = tablecolumn(entries,header)
%TABLECOLUMN  Creates a table column with the specified entries and header
%
% column = tablecolumn(entries,header)
%
% Utility function for the purposes of using a MATLAB list control to display
% tabular data.
%   entries is a cell array of strings. New-lines are replaced with spaces.
%   header is a string
%   column is a cell array of strings ALL OF THE SAME LENGTH.
%     the first entry is the header, then a separator ('-----') and then
%     the entries, padded with trailing spaces to make them the same lengths
% Use a fixed width font and concatentate multiple columns to display the
% tabular data in a list control.
%

entries = strrep(entries,char(10),' '); % replace new-lines with spaces
column = [ { header ; '-' } ; entries(:) ];
column = equalise_lengths(column);
column{2} = strrep(column{2},' ','-');

