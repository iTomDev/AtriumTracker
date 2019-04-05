% Thomas Pile
% August 2018
% Requirements: Matlab 2018a, Computer Vision Toolbox. May work with earlier versions

% What it does:
% 1. Attempts to build a path of people walking through the Atrium so a
% count can be made
% 2. Places markers on screen for the location and estimate. Displays count
% on screen
% 3. Generates a video with the above info in it


% findings
% V1
% 1. NN performs quite well if confidence threshold is fixed at 2 ish, and
% the image size is scaled to 0.5
% 2. NN body detector still works if frame differencing is applied, so long
% as its not too extreme. A too wide a difference makes it tend to ghost.
% Also deblurring does not help. 
% 3. A significant benefit of using frame differencing vs NN with a
% plain image, is that the frame differencing allows the confidence
% threshold to be dropped to that point that with a normal image the
% results would be unusable. This means higher update rates can be
% obtained, fewer missing tracks, and ideally better tracking
% V2
% 4. Moving from skip 2 frames to skip 4 frames and applying a median
% filter means the detector threshold can be reduced to 0.3
% 5. Deblurring is computationally expensive and doesn't give any real
% improvement.
% V4
% 6. Detects stationary people using a face detector and a different
% counting method based on quantity and distribution of detections
r2d = 57.2958;

addyv1 = '227_04_10_17_0002.mp4';
addyv2 = '02_10_17_0004.mp4';
%addyv3 = 'D:\Atrium_Tracker_Vids\CCTV_235\CCTV_235-20180802T211653Z-001\CCTV_235\10_04_18_0000.mp4';
%addyv3 = 'C:\Users\Tom\Desktop\_atrium_tracker_august_2018\CTV_235\10_04_18_0000.mp4';
framecount = 0;

v1 = VideoReader(addyv1);

skip = 0; % always zero`
skipqty = 4; %2 is standard;
% 2 performs well with frame differncing, 3 not as well
% 3 seems better for NNs
v1fcolour1 = [];
v1fcolour2 = [];

% for differencing
picold = zeros(540, 960,'uint8'); 
pic = zeros(540, 960,'uint8'); 
FB = zeros(540, 960,5,'uint8'); 

% generate video output
vo = VideoWriter('newfile.avi','Motion JPEG AVI');
vo.Quality = 95;
open(vo);

% setup the object tracker / counter
clear('Track');

% test objects
obj = [1 1
       50 1];
% initial track value
Track(1).Xpos = -9999;
Track(1).Ypos = -9999;
Track(1).Xvel = 0;
Track(1).Yvel = 0;
Track(1).Velvect = 10;
Track(1).Phi = 0;
Track(1).Dphi = 0;
Track(1).Pvel = 0;
Track(1).Pphi = 0;
Track(1).Historic = 0;
Track(1).StaticCount = 0;
Track(1).TimeSinceUpdate = 0;
Track(1).JourneyLength = 0;
Track(1).Total = 0;

% stationary object counting
Hits = zeros(540,960,1);
StaticCount(1).Count = Hits;
StaticCount(1).Countdown = 0;
StaticCount(1).Total = 0;
% static tracking objects
StaticTrack(1).Location = [0 0];
StaticTrack(1).CountOut = 0;
StaticTrack(1).CumulativeHits = 1;
StaticTrack(1).Historic = 0;
StaticTrack(1).Valid = 0;

% Add known false locations here
Blacklist = [721, 332];

% COD setup
% built in bodu
% bodyDetector = vision.CascadeObjectDetector('UpperBody'); 
% bodyDetector.MinSize = [60 60];
% bodyDetector.MergeThreshold = 10;
%
% 100% image size
detector = vision.PeopleDetector('UprightPeople_128x64');
detector.ClassificationThreshold = 1;
%detector.ScaleFactor = 3;
detector.MinSize = [250 125]; % full size frame

% 50% image size - performs ok
detector = vision.PeopleDetector('UprightPeople_96x48');
detector.MinSize = [128 64]; % 50% frame
%detector.MaxSize = [250 125];
detector.MaxSize = [200 100];
detector.ClassificationThreshold = 2;
%detector.ClassificationThreshold = 0.5; % Jan 2019

% detector for static people
staticdetector = vision.PeopleDetector('UprightPeople_96x48');
staticdetector.MinSize = [128 64]; % 50% frame
%detector.MaxSize = [250 125];
staticdetector.MaxSize = [200 100];
staticdetector.ClassificationThreshold = 1.7;

% face method for detecting static people
%staticdetector = vision.CascadeObjectDetector('UpperBody'); 
%staticdetector = vision.CascadeObjectDetector('UpperBody'); 
%staticdetector = vision.CascadeObjectDetector('UpperBody'); 
staticdetector = vision.CascadeObjectDetector('ProfileFace'); 

% custom detector
%detector = vision.CascadeObjectDetector('atriumPeopleDetector2.xml');
figure; 
ibbox = 1;
start = cputime;

% loop through all frames
while hasFrame(v1) 
    % skip frames
    if isempty(v1fcolour1)==1
        skip = skip+1;
        v1fcolour1 = readFrame(v1);
    else
        if isempty(v1fcolour2)==1
            skip = skip+1;
            v1fcolour2 = readFrame(v1);
        end
    end
    
if isempty(v1fcolour1)==0 || isempty(v1fcolour2)==0
    if isempty(v1fcolour1)==0
        imrgb = imresize(v1fcolour1,0.5);
        v1f = rgb2gray(rgb2ycbcr(imrgb));
        v1fcolour = v1fcolour1;
        v1fcolour1 = [];
    else
        if isempty(v1fcolour2)==0
            skip = skip+1;
            imrgb = imresize(v1fcolour2,0.5);
            v1f = rgb2gray(rgb2ycbcr(imrgb));
            v1fcolour = v1fcolour2;
            v1fcolour2 = [];
        end
    end
        
    %imrgb = imresize(v1fcolour,0.5);
    %v1f = rgb2gray(rgb2ycbcr(imrgb));
    
    % convert to greyscale
    
    % add frame to buffer, shift old frame out. The effect is this:
    % [1 2 3 4]  5 6 7 8
    % 3: 3-1, 4-2, 5-3, etc
    % 4: 4-1, 5-2, 6-3. 7-4, 8-1
    FB(:,:,1) = FB(:,:,2);
    FB(:,:,2) = FB(:,:,3);
    FB(:,:,3) = FB(:,:,4);
    FB(:,:,4) = FB(:,:,5);
    FB(:,:,5) = v1f;
    
    % precharge the buffer
    if framecount>4
        % apply differencing operation to each pair in the buffer
        % for fi=1:4 % all phases for all differences 1 to 4 (slow!)
        % for fi=4 % just the four phases of 4 frame differencing
        delta = 1/4;
        for fi=4
            
            % sitting people detection
            %staticbbox = step(staticdetector,v1f);
            staticbbox = step(staticdetector,imrgb);
            % put X,Y positions into array for tracker
            obj = zeros(size(staticbbox,1), 2);
            obj(:,:) = staticbbox(:,1:2);
            % update tracker 
            [StaticTrack, StaticCount] = staticTracker(obj, StaticTrack, StaticCount);
            % display results
            %detectedImg = insertObjectAnnotation(imrgb,'rectangle',bbox2,'Person','Color','Green');
            %StaticCount(1).Total
            %imshow(detectedImg)
            
            % moving people detection
            pic = FB(:,:,fi);
            picold = FB(:,:,1);
            %pic = rgb2gray(rgb2ycbcr(imresize(v1fcolour,0.5)));
            %pic2 = pic; % so we can save it as picold later
            pic = imadjust(pic-picold, [0.01 0.2], []); % frame difference
            pic = medfilt2(pic);
            bbox = step(detector,pic);
            
            % check points against blacklist
            szBB2 = 0;
            bbox2 = zeros(size(bbox));
            for m=1:size(bbox,1)
                for k=1:size(Blacklist,1)
                    if sqrt( (Blacklist(k,1)-bbox(m,1))^2 + (Blacklist(k,2)-bbox(m,2))^2 ) < 5
                        % remove from bbox
                    else
                        % the value if good, keep
                        szBB2 = szBB2 +1;
                        bbox2(szBB2,:) = bbox(m,:);
                    end
                end
            end
            bbox2;
            %clear('bbox');
            %bbox = zeros(szBB2,size(bbox2,2));
            %bbox(:,:) = bbox2(1:szBB2,:);

    %             % Label them better
    %             label_str = cell(3,1);
    %             conf_val = [85.212 98.76 78.342];
    %             % [x y width height].
    %             position = [23 373 60 66;35 185 77 81;77 107 59 26];
    %             for ii=1:3
    %                 label_str{ii} = ['Confidence: ' num2str(conf_val(ii),'%0.2f') '%'];
    %             end
    %             RGB = insertObjectAnnotation(I,'rectangle',position,label_str,...
    %                 'TextBoxOpacity',0.9,'FontSize',18);
    %             imshow(RGB)

            % put X,Y positions into array for tracker
            obj = zeros(size(bbox,1), 2);
            obj(:,:) = bbox(:,1:2);

            % update tracker 
            [x, Track] = runTracker2(obj, Track, delta);
            % get Trackers estimates for plotting
            LAR = getTrackEstimates(Track,obj);

            % how many people so far
            size(Track,2)
            getTotalCount(Track)
            
            % label them better
            label_str = cell(size(bbox,1),1);
            %conf_val = [85.212 98.76 78.342];
            index = 1:1:size(bbox,1);
            % [x y width height].
            %position = [23 373 60 66;35 185 77 81;77 107 59 26];
            position = bbox(:,1:4);
            for ii=1:size(bbox,1)
                label_str{ii} = ['Count: ' num2str(index(ii),'%0.2f')];
            end
            if size(bbox,1)==0 % nothing detected
                % as detected
                % pic: greyscale
                % imrgb: colour
                detectedImg = insertObjectAnnotation(imrgb,'rectangle',staticbbox,'Sitting Person','Color','Blue');
                detectedImg = insertObjectAnnotation(detectedImg,'rectangle',bbox,'Person','Color','Green');
                % project forward
                
                % loop though the projected locations and plot them
                for pl=1:size(LAR,1)
                    detectedImg = insertObjectAnnotation(detectedImg,'circle',LAR(pl,:),['Est: ' num2str(pl,'%d')] );
                end
                
                % display the total count so far
                count_str = ['Total Moving People: ', num2str(Track(1).Total,'%d')];
                detectedImg = insertText(detectedImg,[10 10],count_str,'FontSize',18,'BoxColor',...
                'green','BoxOpacity',0.4,'TextColor','white');
                % display the total of sitting people so far
                count_str = ['Total Sitting People: ', num2str(StaticCount(1).Total,'%d')];
                detectedImg = insertText(detectedImg,[10 40],count_str,'FontSize',18,'BoxColor',...
                'blue','BoxOpacity',0.4,'TextColor','white');
                % show final image
                imshow(detectedImg);
            else % something detected
                % as detected
    %             RGB = insertObjectAnnotation(pic,'rectangle',position,label_str,...
    %                 'TextBoxOpacity',0.9,'FontSize',18);
    %             imshow(RGB)

                % pic: greyscale
                % imrgb: colour
                % as detected alt
                %detectedImg = insertObjectAnnotation(imrgb,'rectangle',bbox,'Person','Color','Green');
                detectedImg = insertObjectAnnotation(imrgb,'rectangle',staticbbox,'Sitting Person','Color','Blue');
                detectedImg = insertObjectAnnotation(detectedImg,'rectangle',bbox,'Person','Color','Green');
                % display the total of moving people so far
                count_str = ['Total People: ' num2str(Track(1).Total,'%d')];
                detectedImg = insertText(detectedImg,[10 10],count_str,'FontSize',18,'BoxColor',...
                'green','BoxOpacity',0.4,'TextColor','white');
                % display the total of sitting people so far
                count_str = ['Total Sitting People: ' num2str(StaticCount(1).Total,'%d')];
                detectedImg = insertText(detectedImg,[10 40],count_str,'FontSize',18,'BoxColor',...
                'blue','BoxOpacity',0.4,'TextColor','white');
            
                % loop though the projected locations and plot them
                for pl=1:size(LAR,1)
                    detectedImg = insertObjectAnnotation(detectedImg,'circle',LAR(pl,:),['Est: ' num2str(pl,'%d')] );
                end
                
                % show final image 
                imshow(detectedImg);

                % with colour image instead
                RGB = insertObjectAnnotation(pic,'rectangle',position,label_str,...
                    'TextBoxOpacity',0.9,'FontSize',18);
                %imshow(RGB)
            end
            
            % write video out
            %writeVideo(vo,detectedImg)

            % diplay results with false positives removed
            %detectedImg = insertObjectAnnotation(pic,'rectangle',bbox,'Person');
            %imshow(detectedImg);


            % collect all points, just for debugging
            if size(bbox,1)>0
                bboxsamples(ibbox).bbox = bbox;
                ibbox = ibbox + 1;
            end

            % end of processing that frame
            %picold = pic2;

        end
    end
    framecount = framecount + 1;
end
end





