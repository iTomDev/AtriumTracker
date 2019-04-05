function [count] = getTotalCount(Track)
%GETTOTALCOUNT Returns total count
% Loops through the Track object and counts the number of targets marked
% historic minus 1 for the zero target used to define the structure.

count = 0;
for i=1:size(Track,2)
    if Track(i).Historic==1
        count = count + 1;
    end
end

end

