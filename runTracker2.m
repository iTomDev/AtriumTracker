function [ret, Track] = runTracker(obj,Track,delta)
%runTracker: Object tracker and counter for the image processing toolbox.
%   Provided an array of object locations at discrete time steps
%the function will attempt to track the objects movement, discriminate
%between objects, and count the total objects to pass through the scene.
% Thomas Pile, August 2018

% Key points
% + kalman filters to track direction and velocity
% + measurements are compared with estimated (projected forward) position
% for each exising track
% + tracks are timed out of the frame
% + measure journey length to determine which objects are less valid

% References
% Kalman filter (descriptive)
% http://web.mit.edu/kirtley/kirtley/binlustuff/literature/control/Kalman%20filter.pdf
% Kalman filer (source)
% R. E. Kalman, “A new approach to linear filtering and prediction problems,” J. Basic Eng., vol. 82, no.1, pp. 35–45, Mar. 1960. 

r2d = 57.2958;

% tuning values

% This determines how far off a point can be, and still be considered
% as associated with an existing target track. If its outside this
% range a new track is generated.
% Bigger: May misassign targets
% Smaller: May lose tracks
target_assign_threshold = 300; %300;
% best guess for velocity
velocityguess = 5;
% corners of the screen [(xl,yl),(xr,yr)]
box = [1,1,540,960];
% number of frames before a frame near the edge has "gone"
drop_track_timeout = 5;
% distance from the edge at which we apply this
prox_track_timeout = 5;

%persistent Track
ret = 1;


if isempty(Track)
    %X = 0; % state vector
    %P = 99999; % covariance

    % Objects are identified by row number
    %         X,    Y      vel,   phi,   historic, static_count, dphi, Pvel, Phi
    %         1     2      3      4      5         6             7     8     9
    %Track = [-9999, -9999, -9999, -9999, 0,        0,            0,    9999, 9999]; 
    Track(1).Xpos = -1;
    Track(1).Ypos = -1;
    Track(1).Xvel = 0;
    Track(1).Yvel = 0;
    Track(1).Velvect = 0;
    Track(1).Phi = 0;
    Track(1).Dphi = 0;
    Track(1).Pvel = 0;
    Track(1).Pphi = 0;
    Track(1).Historic = 0;
    Track(1).StaticCount = 0;
    Track(1).TimeSinceUpdate = 0;
    Track(1).JourneyLength = 0;
    Track(1).Total = 0;

    % Note that 1 is basically an empty ex and should NEVER be matched
end

% increase the time since update flag for all targets
for k=1:size(Track,2)
    Track(k).TimeSinceUpdate = Track(k).TimeSinceUpdate + 1/delta;

    % if update count exceeds a threshold, mark as historical
    if Track(k).TimeSinceUpdate > 120 %30
        Track(k).Historic = 1;
    end
end

% Find which target in the target vector is most likely to be it
for i=1:size(obj,1)% loop through all objects


    % estimate which target is closest
    % loop through target vector
    bestmatch = [];
    targeterrprev = 9999999999999;
    for j=1:size(Track,2)
        % alternative method
        ang = Track(j).Phi;
        %ang = atan2( Track(j).Ypos, Track(j).Xpos);
        projectedmag = (Track(j).TimeSinceUpdate*Track(j).Velvect);
        [projectedX projectedY] = pol2car(projectedmag,ang);
        projectedX = Track(j).Xpos + projectedX;
        projectedY = Track(j).Ypos + projectedY;

        % determine an adaptive threshold for how close is close
        % enough. Should depend on the time target has had to deviate
        %target_assign_threshold = 10+TimeSinceUpdate

        % error between the measurement and the estimate position
        targeterr = sqrt( (obj(i,1)-projectedX)^2 + (obj(i,2)-projectedY)^2 );
        % angle between last known point and here
        estangle = atan2((obj(i,2)-Track(j).Ypos), (obj(i,1)-Track(j).Xpos));

        targeterr;
        disp_est_angle = estangle*r2d;
        %angle_error = (estangle-Track(j).Phi)*r2d
        %dis_TrackAngle = Track(j).Phi*r2d


        % find index of the smallest
        if targeterr<targeterrprev
            % check that its within reasonable count
            if(targeterr<target_assign_threshold)
                % check that difference between measurement and
                % past point Track(j) in terms of angle is less
                % than some value
                %if abs(estangle)*r2d < 90
                    % make sure it isn't a track that has left the scene
                    if Track(j).Historic==0
                        % check that the direction of travel is vaguely similar
                        % helps with crossing people
                        % skipped for now
                        bestmatch = j;
                    end
                %end
            end
        end
        targeterrprev = targeterr;
    end

    %projectedX-Track(j).Xpos
    %projectedX 
    %projectedY

    % at the moment a result is always returned from above
    if isempty(bestmatch)==1 || exist('bestmatch','var')==0
        % no target table match was found, add as new
        ni = size(Track,2)+1;
        Track(ni).Xpos = obj(i,1);
        Track(ni).Ypos = obj(i,2);
        Track(ni).Xvel = 0;
        Track(ni).Yvel = 0;
        Track(ni).Velvect = velocityguess;
        Track(ni).Phi = 0;
        Track(ni).Dphi = 0;
        Track(ni).Pvel = 0;
        Track(ni).Pphi = 0;
        Track(ni).Historic = 0;
        Track(ni).StaticCount = 0;
        Track(ni).TimeSinceUpdate = 0;
        Track(ni).JourneyLength = 0;
        Track(ni).Total = 0;
    else
        % make of probable target reference easier to handle
        bm = bestmatch;

        % hopefully improved best guess for angle for faster convergence
        if Track(bm).JourneyLength==0
            Track(bm).Phi = atan2( (obj(i,2)-Track(bm).Ypos), (obj(i,1)-Track(bm).Xpos));
        end

        % Kalman filter for magnitude
        A = 1; % state transition matrix
        H = 1; % connect measurement to state 
        R = 0.6; % measurement noise estimate
        Q = 1.0; % bigger this, quicker convergence
        % set vals for kf
        X = Track(bm).Velvect;
        P = Track(bm).Pvel;
        % distance travelled
        z = sqrt( (obj(i,1)-Track(bm).Xpos)^2 + (obj(i,2)-Track(bm).Ypos)^2 );
        % update journey length
        Track(bm).JourneyLength = Track(bm).JourneyLength + z;
        % get velocity est
        if Track(bm).TimeSinceUpdate >0 % else inf
            z = z/Track(bm).TimeSinceUpdate; 
        end
        % state update
        Xn = A*X; % project forward state
        Pn = A*P*A'+Q; % project forward covariance
        % correct estimate
        % calculate kalman gain
        K = (Pn*H')/(H*Pn*H'+R); % calculate kalman gain "/" for inv
        X = Xn + K*(z-H*Xn); % calculate state
        P = (1-K*H)*Pn; % calvulate covariance
        % velocity estimates
        Track(bm).Velvect = X;
        Track(bm).Pvel = P;

        % increase static count if its not moving
        % this is used to determine if a target has left. It does not
        % determine if the target is stationary, we dont care
        if Track(bm).Velvect==0
            Track(bm).StaticCount = Track(bm).StaticCount + 1;
        end

        % Kalman filter for direction
        A = 1; % state transition matrix
        H = 1; % connect measurement to state 
        R = 0.5; % measurement noise estimate
        Q = 1.1; % % bigger this, quicker convergence
        % set vals for kf
        X = Track(bm).Phi; % best guess is current angle
        P = Track(bm).Pphi;
        anglemeas = atan2( (obj(i,2)-Track(bm).Ypos), (obj(i,1)-Track(bm).Xpos) );
        z = anglemeas; % measured from vector
        % state update
        Xn = A*X; % project forward state
        Pn = A*P*A'+Q; % project forward covariance
        % correct estimate
        % calculate kalman gain
        K = (Pn*H')/(H*Pn*H'+R); % calculate kalman gain "/" for inv
        X = Xn + K*(z-H*Xn); % calculate state
        P = (1-K*H)*Pn; % calvulate covariance
        % directiom estimates
        Track(bm).Phi = X;
        Track(bm).Pphi = P;
        %anglemeas;

        % Update state with info
        % location from latest measurement
        Track(bm).Xpos = obj(i,1);
        Track(bm).Ypos = obj(i,2);
        %Track(bm).Dphi = Xn-X; %dphi
        % zero its time since update counter
        Track(bm).TimeSinceUpdate = 0;


        % increase the update flag

        % increase the time counter (how long transiting)
        X;
        %P % plot this!
    end

end


% calculate total
Track(1).Total = 0;
for i=1:size(Track,2)
    if Track(i).JourneyLength > 200
        Track(1).Total = Track(1).Total + 1;
    end
end

%     % loop through tracks
%     for j=1:size(Track,1)
%         % Determine which have left the scene, then mark historical
%         % if it is near the edge and not been updated, its gone
%         if(Track(j,6)>drop_track_timeout)
%             % check if its near the edge
%             if Track(j,1)<box(1)+prox_track_timeout || Track(j,1)>box(3)-prox_track_timeout % in X
%                 if Track(j,2)<box(2)+prox_track_timeout || Track(j,2)>box(4)-prox_track_timeout % in Y
%                     % its near the edge, and not moving, declare historical
%                     Track(j,5) = 1;
%                 end
%             end
%         end
%         % count total so far
%     end
end

