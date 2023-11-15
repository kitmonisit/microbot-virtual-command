%% HGTable Example
% Uses the "hgtable" functions to display a multi-column list
%% Create figure and list box
f = figure('Name','HGTable Example','NumberTitle','off','Visible','off');
b = uicontrol('parent',f,'style','listbox',...
    'Units','Normalized','Position',[0.1 0.1 0.8 0.8]);
% Create local anonymous functions for callbacks from the list.
opencb = @(src,evt) fprintf(1,'Item %d opened\n',evt);
selectioncb = @(src,evt) fprintf(1,'Item %d selected\n',evt);
% Set the properties of the list box, including font, selection behaviour,
% and callbacks.
tablesetup(b,selectioncb,opencb);

%% Create sample data from properties of list box
% Each column needs to be a cell array of strings.  We'll have columns
% for: property name, property data type, the size of the value for that
% property, and lastly the value itself (if it is a string).
p = get(b);
propnames = fieldnames(p);
datatype = cell(size(propnames));
datasize = cell(size(propnames));
stringdata = cell(size(propnames));
for i=1:numel(propnames)
    v = get(b,propnames{i});
    datatype{i} = class(v);
    s = size(v);
    datasize{i} = sprintf('%d*%d',s(1),s(2));
    if ischar(v)
        stringdata{i} = v;
    else
        stringdata{i} = '<not a string>';
    end
end

%% Display data in list box
% The function "formattable" does everything for us.  We could, if
% necessary, format columns individually using "tablecolumn", join
% them together using "tablestrings", and set that as the string for
% the list box.
formattable(b,{'Property Name','Data Type','Data Size','Data'},...
    propnames,datatype,datasize,stringdata);
set(f,'Visible','on');

%% Click and/or double-click items in the list to see the callbacks execute.

