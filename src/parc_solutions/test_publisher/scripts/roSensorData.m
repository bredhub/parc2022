% Define the callback functions

function lidarCallback(~, data)
    global detectedList robotStatus sampleWeedImage
    
    if strcmp(robotStatus, 'started')
        detectedObjects = extractDetectedObjects(data);
        filteredObjects = filterWeedObjects(detectedObjects);
        detectedList = [detectedList; filteredObjects];
    end
end

function detectedObjects = extractDetectedObjects(data)
    detectedObjects = [];
    angleMin = data.AngleMin;
    angleIncrement = data.AngleIncrement;
    
    for i = 1:numel(data.Ranges)
        distance = data.Ranges(i);
        if distance < 1.0
            angle = angleMin + (i-1) * angleIncrement;
            x = distance * cos(angle);
            y = distance * sin(angle);
            detectedObjects = [detectedObjects; x, y];
        end
    end
end

function filteredObjects = filterWeedObjects(detectedObjects)
    global sampleWeedImage
    
    filteredObjects = [];
    threshold = 0.8;
    sampleWeedImageGray = rgb2gray(sampleWeedImage);
    
    for i = 1:size(detectedObjects, 1)
        obj = detectedObjects(i, :);
        objResized = imresize(obj, [size(sampleWeedImageGray, 1), size(sampleWeedImageGray, 2)]);
        
        % Perform template matching
        result = normxcorr2(objResized, sampleWeedImageGray);
        
        % Find the maximum correlation value
        [maxValue, ~] = max(result(:));
        
        if maxValue >= threshold
            filteredObjects = [filteredObjects; obj];
        end
    end
end

function leftCameraCallback(~, data)
    global detectedList robotStatus sampleWeedImage
    
    if strcmp(robotStatus, 'started')
        cvImage = reshape(typecast(data.Data, 'uint8'), [data.Height, data.Width, data.Step]);
        matchedObjects = compareWithSample(cvImage, sampleWeedImage);
        detectedList = [detectedList; matchedObjects];
    end
end

function matchedObjects = compareWithSample(image, sampleWeedImage)
    matchedObjects = [];
    threshold = 0.22;
    imageGray = rgb2gray(image);
    sampleWeedImageGray = rgb2gray(sampleWeedImage);
    
    % Perform template matching
    result = normxcorr2(sampleWeedImageGray, imageGray);
    
    % Find the locations where the correlation exceeds the threshold
    [rows, cols] = find(result >= threshold);
    
    % Append the matched object coordinates to the list
    for i = 1:numel(rows)
        matchedObjects = [matchedObjects; cols(i), rows(i)];
    end
end

function rightCameraCallback(~, data)
    global detectedList robotStatus sampleWeedImage
    
    if strcmp(robotStatus, 'started')
        cvImage = reshape(typecast(data.Data, 'uint8'), [data.Height, data.Width, data.Step]);
        matchedObjects = compareWithSample(cvImage, sampleWeedImage);
        detectedList = [detectedList; matchedObjects];
    end
end

function zedCameraCallback(~, data)
    global detectedList robotStatus sampleWeedImage
    
    if strcmp(robotStatus, 'started')
        cvImage = reshape(typecast(data.Data, 'uint8'), [data.Height, data.Width, data.Step]);
        matchedObjects = compareWithSample(cvImage, sampleWeedImage);
        detectedList = [detectedList; matchedObjects];
    end
end

function gpsCallback(~, data)
    global detectedList
    
    if strcmp(robotStatus, 'started')
        [x, y] = gpsToCartesian(data.Latitude, data.Longitude);
        lastDetectedObjectIndex = size(detectedList, 1);
        detectedList(lastDetectedObjectIndex, 3:4) = [x, y];
    end
end

function robotStatusCallback(~, data)
    global robotStatus
    
    robotStatus = data.Data;
    
    if strcmp(robotStatus, 'started')
        disp('Robot status: Started');
    elseif strcmp(robotStatus, 'finished')
        disp('Robot status: Finished');
        disp('Detected objects:');
        disp(detectedList);
        disp('Number of detected objects:');
        disp(size(detectedList, 1));
        disp('Exiting...');
        clear global;  % Clear global variables
        return;
    end
end

% Set up the necessary variables

global detectedList robotStatus sampleWeedImage

% Global flag to track the robot status
robotStatus = 'finished';

% Path to the sample weed image
sampleWeedImagePath = 'path/to/sample/weed/image.png';

% Load the sample weed image
sampleWeedImage = imread(sampleWeedImagePath);

% Initialize the detected list
detectedList = [];

% Set up ROS and subscribers

rosinit;

% Subscribe to LiDAR sensor
lidarSub = rossubscriber('/scan');
lidarSub.NewMessageFcn = @lidarCallback;

% Subscribe to left and right cameras
leftCameraSub = rossubscriber('/left_camera/image_raw');
leftCameraSub.NewMessageFcn = @leftCameraCallback;

rightCameraSub = rossubscriber('/right_camera/image_raw');
rightCameraSub.NewMessageFcn = @rightCameraCallback;

% Subscribe to ZED camera topics
zedLeftCameraSub = rossubscriber('/zed2/left/image_rect_color');
zedLeftCameraSub.NewMessageFcn = @zedCameraCallback;

zedRightCameraSub = rossubscriber('/zed2/right/image_rect_color');
zedRightCameraSub.NewMessageFcn = @zedCameraCallback;

% Subscribe to GPS sensor
gpsSub = rossubscriber('/gps');
gpsSub.NewMessageFcn = @gpsCallback;

% Subscribe to robot status topic
robotStatusSub = rossubscriber('/parc_robot/robot_status');
robotStatusSub.NewMessageFcn = @robotStatusCallback;

% Wait until the robot finishes
while ~strcmp(robotStatus, 'finished')
    pause(1);  % Add a small delay to avoid high CPU usage
end

% Clean up

rosshutdown;
