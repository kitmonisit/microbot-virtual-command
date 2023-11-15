function strx = tablestrings(varargin)
%TABLESTRINGS Combines data returned by tablecolumn into a single array
%
% strx = tablestrings(column1,column2,...)
%
% The columns are joined horizontally, with entries separated by "|" characters.
% The resulting cell array of strings can then be set as the "string" property
% in a MATLAB list control.  If the control uses a fixed-width font, the
% data will be displayed in a tidy tabular form.

% the joining strings need to be cells too, or strcat strips out
% white-space.
join = {'| '};

catargs = cell(2*nargin - 1,1);

for i=1:nargin
    catargs{2*i - 1} = varargin{i};
    if i<nargin
        catargs{2*i} = join;
    end
end

strx = strcat({'  '},catargs{:});

