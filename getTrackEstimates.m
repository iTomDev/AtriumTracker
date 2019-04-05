function [LAR] = getTrackEstimates(Track2, obj)
%GETTRACKESTIMATES Returns the projected forward location and radius for 
% all Track items.
% Returns LAR: Location and radius [for plotting]
% This can be used to check how accurate the tracker is visually
target_assign_threshold = 150; 

LAR = zeros(size(Track2,2), 3);

if size(Track2,2)>0
    for j=1:size(obj,1)
        for i=1:size(Track2,2)
            % run the estimate used by tracker
            ang = atan2( Track2(i).Ypos, Track2(i).Xpos);
            ang = atan2((obj(j,2)-Track2(i).Ypos), (obj(j,1)-Track2(i).Xpos));
            projectedmag = (Track2(i).TimeSinceUpdate*Track2(i).Velvect);
            [projectedX projectedY] = pol2car(projectedmag,ang);
            projectedX = Track2(i).Xpos + projectedX;
            projectedY = Track2(i).Ypos + projectedY;
            LAR(i,1) = projectedX;
            LAR(i,2) = projectedY;
            LAR(i,3) = target_assign_threshold;
        end
    end
else    
    LAR = [0 0 0];
end


end

