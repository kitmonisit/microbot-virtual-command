function formattable(listbox,headers,varargin)
%FORMATTABLE Formats header and column data and displays it in a listbox
%
% formattable(listbox,headers,column1,column2,...)
%
% The first input must be a listbox handle.
% All other arguments are cell arrays of strings, where:
%   headers contains the title for each column, and
%   further arguments contain the data for each column.
% The number of header entries must equal the number of columns,
% and the length of each column must be the same.

ASSERT(ishandle(listbox),'Valid listbox handle required');

ncols = nargin-2;

ASSERT(length(headers)==ncols,'One header for every column');


cols = cell(ncols,1);

for i=1:ncols
    cols{i} = tablecolumn(varargin{i},headers{i});
end

strx = tablestrings(cols{:});

set(listbox,'string',strx,'val',1);

