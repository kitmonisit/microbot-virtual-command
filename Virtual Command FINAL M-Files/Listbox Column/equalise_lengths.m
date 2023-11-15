function c = equalise_lengths(s)
%EQUALISE_LENGTHS Makes every string in a cell array the same length. 
%
%  c = equalise_lengths(s);
%
%  s is a cell array of strings of different lengths
%  c is a cell array of strings, all of the same length.
%  Shorter strings are padded with trailing spaces.
%
%  See also: cellstr.

s = char(s); % this turns s into a char matrix with trailing blanks in each row

c = cell(size(s,1),1); % for each row in s
for i=1:size(s,1)
    c{i} = s(i,:); % put the row into a cell in c
end

