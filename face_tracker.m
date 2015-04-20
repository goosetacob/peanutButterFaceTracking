% if(~isdeployed)
%         cd(fileparts(which(mfilename)));
% end
% clc; % Clear command window.
% clear; % Delete all variables.
% close all; % Close all figure windows except those created byimtool.
% imtool close all; % Close all figure windows created by imtool.
% workspace; % Make sure the workspace panel is showing.
% fontSize = 36;
% 
% % Start up the video.
% vid = videoinput('macvideo', 2, 'ARGB32_1024x768');
% vid.FramesPerTrigger = 1;
% 
% preview(vid);
% start(vid);
% 
% % Get image and display it.
% rgbImage = getdata(vid);
% hsvImage = rgb2hsv(rgbImage);
% imshow(hsvImage);
% [ Y X ] = ginput(1);
% close all;
% imtool close all;
% rgbImage_Hue = rgbImage(round(X),round(Y),1);
% rgbImage_Sat = rgbImage(round(X),round(Y),2);
% stop(vid);

%Init arduino and x,y servos
a = arduino();
s_y = servo(a, 4, 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);
s_x = servo(a, 5, 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);
angle = 0;

% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);

% Create the webcam object.
cam = webcam('Logitech Camera');

% Capture one frame to get its size.
videoFrame = snapshot(cam);
frameSize = size(videoFrame);

% Create the video player object.
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

runLoop = true;
numPts = 0;
frameCount = 0;
old_angleY = .5
old_angleX = .5;

while runLoop && frameCount < 1000

    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;

    if numPts < 10
        % Detection mode.
        bbox = faceDetector.step(videoFrameGray);
        writePosition(s_x, double(old_angleX));
        writePosition(s_y, double(old_angleY));
        if ~isempty(bbox)
            % Find corner points inside the detected region.
            points = detectMinEigenFeatures(videoFrameGray, 'ROI', bbox(1, :));

            % Re-initialize the point tracker.
            xyPoints = points.Location;
            numPts = size(xyPoints,1);
            release(pointTracker);
            initialize(pointTracker, xyPoints, videoFrameGray);

            % Save a copy of the points.
            oldPoints = xyPoints;

            % Convert the rectangle represented as [x, y, w, h] into an
            % M-by-2 matrix of [x,y] coordinates of the four corners. This
            % is needed to be able to transform the bounding box to display
            % the orientation of the face.
            bboxPoints = bbox2points(bbox(1, :));

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);

            % Display a bounding box around the detected face.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);

            % Display detected corners.
            videoFrame = insertMarker(videoFrame, xyPoints, '+', 'Color', 'white');
        end

    else
        % Tracking mode.
        [xyPoints, isFound] = step(pointTracker, videoFrameGray);
        visiblePoints = xyPoints(isFound, :);
        oldInliers = oldPoints(isFound, :);

        numPts = size(visiblePoints, 1);

        if numPts >= 10
            % Estimate the geometric transformation between the old points
            % and the new points.
            [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
                oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

            % Apply the transformation to the bounding box.
            bboxPoints = transformPointsForward(xform, bboxPoints);

            % Convert the box corners into the [x1 y1 x2 y2 x3 y3 x4 y4]
            % format required by insertShape.
            bboxPolygon = reshape(bboxPoints', 1, []);
            
            %Calculate center of bounding box
            widthX = max([bboxPolygon(1) bboxPolygon(3) bboxPolygon(5) bboxPolygon(7)]) - min([bboxPolygon(1) bboxPolygon(3) bboxPolygon(5) bboxPolygon(7)]);
            widthY = max([bboxPolygon(2) bboxPolygon(4) bboxPolygon(6) bboxPolygon(8)]) - min([bboxPolygon(2) bboxPolygon(4) bboxPolygon(6) bboxPolygon(8)]);
            
            faceCenterX = widthX/2 + min([bboxPolygon(1) bboxPolygon(3) bboxPolygon(5) bboxPolygon(7)])
            faceCenterY = widthY/2 + min([bboxPolygon(2) bboxPolygon(4) bboxPolygon(6) bboxPolygon(8)])
            
            %Calculate center of frame
            frameCenterX = size(videoFrame,2)/2;
            frameCenterY = size(videoFrame,1)/2;
            
            %Difference to center bounding box
            res = 3;
            diffX = frameCenterX - faceCenterX
            diffY = frameCenterY - faceCenterY
           
            %Proportion  of face to camera
            k_p_y = 0.6;
            k_p_x = 0.6;
            
            %SOME ARDUINO SHIT HAPPENS
            new_angleX =  (((diffX) / (frameCenterX*2)) * k_p_x) + .5;
            new_angleY =  (((diffY) / (frameCenterY*2)) * k_p_y) + .5
            if( new_angleY - old_angleY > 0.0025)
                new_angleY = old_angleY + 0.0025;
            end
            if( new_angleY - old_angleY < 0.0025)
                new_anlgeY = old_angleY - 0.0025;
            end
            if( new_angleX - old_angleX > 0.0025)
                new_angleX = old_angleX + 0.0025;
            end
            if( new_angleX - old_angleX < 0.0025)
                new_anlgeX = old_angleX - 0.0025;
            end
            if( new_angleX > 1)
                new_angleX = 1;
            end
            if( new_angleX < 0)
                new_angleX = 0;
            end
            if( new_angleY > 1)
                new_angleY = 1;
            end
            if( new_angleY < 0)
                new_angleY = 0;
            end
            
            %for i = 1:res
                  writePosition(s_x, double(new_angleX));
                  writePosition(s_y, double(new_angleY));
%                  writePosition(s_x, .5);
%                  writePosition(s_y, .5);
%             %end
             old_angleY = new_angleY;
             old_angleX = new_angleX;
%             % Display a bounding box around the face being tracked.
            videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, 'LineWidth', 3);
% 
%             % Display tracked points.
            videoFrame = insertMarker(videoFrame, visiblePoints, '+', 'Color', 'white');

            % Reset the points.
            oldPoints = visiblePoints;
            setPoints(pointTracker, oldPoints);
        end

    end

    % Display the annotated video frame using the video player object.
    step(videoPlayer, videoFrame);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end

% Clean up.
clear cam;
release(videoPlayer);
release(pointTracker);
release(faceDetector);  