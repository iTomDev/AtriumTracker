function [StaticTrack, StaticCount] = staticTracker(obj, StaticTrack, StaticCount)
%STATICTRACKER Analyses a set of points to count seated people
% The results for seated counting are separate as the techniques required
% are different and one set of results may be more reliable than the other
% Thomas Pile, Sheffield Hallam, August 6th 2018

% principle of operation:
% Repeated detections at the same point are false positives
% Repeated detections within a small radius are seated people
% People walking around produce a distribution around some common area
% Best approach is to log all the points as they come in, then threshold
% them. To keep memory levels under control periodically remove all the low
% level stuff. Also when a validated sitting person drops below the
% threshold for a while consider the seat vacated and be ready to restart
% the count there.

% Static count keeps track of hits on the images over time. This ois
% necessary to threshold and determine islands of hits. It is not a counter
% of people itself. It requires further processing, see StaticTrack.
% Also used for detecting false positives if the same exact location gets
% too many hits
% StaticCount format: 
% StaticCount.Count: m x n x double
% StaticCount.Countdown: Int. When this reaches zero, remove 1 from all counts
% Hits = zeros(540,960,1);
% StaticCount(1).Count = Hits;
% StaticCount(1).Countdown = 0;
% StaticCount(1).Total

% Static track keeps track of the counted islands. This also supports
% counting out vacant seats.
% StaticTrack format: 
% StaticTrack.Location: Estimated location
% StaticTrack.FrameInit: Frame when track was initialised
% StaticTrack.Countout: 
% StaticTrack.CumulativeHits: How many detects were made during the count

for i=1:size(obj)
    StaticCount(1).Count(obj(i,2), obj(i,1)) = StaticCount(1).Count(obj(i,2), obj(i,1)) + 1;
end
% shrink the arry a small amount every time to remove backrground noise (if
% any)
StaticCount(1).Countdown = StaticCount(1).Countdown + 1;
if(StaticCount(1).Countdown > 90)
    StaticCount(1).Countdown = 0;
    % clean up matrix
    StaticCount(1).Count = StaticCount(1).Count .* (StaticCount(1).Count>5);
end

dist_thresh = 50;
VALID_HITS_THRESH = 50;

% check the points provided by obj against the array, are any above the
% threshold
for i=1:size(obj,1)
    % is point near another point?
    mag_thresh = 3; % how many hits before we start counting it
    minmag = 99999999;
    minmagid = 0;
    % find nearest StaticTrack
    for j=1:size(StaticTrack,2)
        if StaticTrack(j).Historic == 0
            mag = abs(sqrt( (obj(i,2)-StaticTrack(j).Location(1))^2 + (obj(i,1)-StaticTrack(j).Location(2))^2 ));
            if mag<minmag
                if mag<dist_thresh
                    minmag = mag;
                    minmagid = j;
                end
            end
        end
    end
    
    % if there is a match
    if minmagid>0
        % there is a similar track nearby
        % increment that track, update its updatetime
        StaticTrack(minmagid).CumulativeHits = StaticTrack(minmagid).CumulativeHits +1;
        StaticTrack(minmagid).CountOut = 0;
    else
        % if enough hits are registered, create new track
        if StaticCount.Count(obj(i,2), obj(i,1)) > mag_thresh
            % create new tracking record
            newi = size(StaticTrack,2) + 1;
            StaticTrack(newi).Location = [obj(i,2), obj(i,1)];
            StaticTrack(newi).CountOut = 0;
            StaticTrack(newi).CumulativeHits = 1;
            StaticTrack(newi).Historic = 0;
            StaticTrack(newi).Valid = 0;
        end

    end
end

% go through tracks and check which are historical, zero out the
% StaticCount location and the radius around it so a new count there can
% begin
% Also refresh the total count
for j=1:size(StaticTrack,2)
    if StaticTrack(j).Historic ==0
        
        % increase the countout value for all tracks
        StaticTrack(j).CountOut = StaticTrack(j).CountOut + 1;
        
        % process counted out tracks
        if (StaticTrack(j).CountOut) > 120 && StaticTrack(j).Historic == 0
            StaticTrack(j).Historic = 1;
            % zero the region around it
            ymin = StaticTrack(j).Location(1)-dist_thresh;
            ymax = StaticTrack(j).Location(1)+dist_thresh;
            if ymin<1 
                ymin=1; 
            end
            if ymax>540
                ymax=540;
            end

            xmin = StaticTrack(j).Location(2)-dist_thresh;
            xmax = StaticTrack(j).Location(2)+dist_thresh;
            if xmin<1 
               xmin=1; 
            end
            if xmax>540
                xmax=540;
            end
            % remove record from array
            %StaticTrack(j) = [];
            %j = j-1;
            StaticCount.Count(ymin:1:ymax,xmin:1:xmax) = 0;
            
        end
        % set it as a valid track if it had enough counts over time
        % if you don't do this then moving tracks will end up in the
        % count
        if StaticTrack(j).CumulativeHits > VALID_HITS_THRESH
            StaticTrack(j).Valid = 1;
        end
    end
end

StaticCount(1).Total = 0;
for j=1:size(StaticTrack,2)
    % increase the total count (more efficient to do it here)
    if StaticTrack(j).Valid >0
        StaticCount(1).Total = StaticCount(1).Total + 1;
    end
end

%{

% plot mesh plot of the StaticTrack object
PlotTrack = ones(540,960);
PlotTrack = flipud(StaticCount.Count);
%PlotTrack = fliplr(PlotTrack);
mesh(1:960,1:540,PlotTrack(:,:))

%}

end


