function tablesetup(listbox,selection_callback,open_callback)
%TABLESETUP Sets up a listbox for use as a table
%
% tablesetup(listbox,selection_callback,open_callback)
%
% listbox must be a handle to a MATLAB listbox.
%
% The other two arguments are "callbacks" in the form used
% by handle-graphics.
%   The "selection_callback" is invoked whenever a new row is
%   selected in the table.
%   The "open_callback" is invoked when a row is double-clicked.
%
% In both callbacks, the "eventdata" parameter is the index of
% the clicked row (not counting the two header rows).  If a
% header row is selected, the index is zero.  If a header
% row is double-clicked, there is no callback.

if exist('selection_callback','var')
    if ~iscell(selection_callback)
        selection_callback = {selection_callback};
    end
else
    selection_callback = [];
end

if exist('open_callback','var')
    if ~iscell(open_callback)
        open_callback = {open_callback};
    end
else
    open_callback = [];
end

set(listbox,'FontName','FixedWidth',...
    'min',0,'max',1,... % single selection
    'callback',{@i_listclick,selection_callback,open_callback});


%%%%%%%%%%%%%%%%%%%%%%%%
function i_listclick(list,evt,selection_callback,open_callback)

fig = get(list,'parent');
val = get(list,'value') - 2; % two header rows
if val<0
    val = 0;
end
if strcmp(get(fig,'selectiontype'),'open')
    if val>0
        if ~isempty(open_callback)
            feval(open_callback{1},list,val,open_callback{2:end});
        end
    end
else
    if ~isempty(selection_callback)
        feval(selection_callback{1},list,val,selection_callback{2:end});
    end
end



