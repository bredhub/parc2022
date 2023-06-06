% ROS initialization
rosinit;

% Create a subscriber to /parc_robot/robot_status topic
robotStatusSub = rossubscriber('/parc_robot/robot_status', 'std_msgs/String');

% Create a publisher to /parc_robot/weed_detection topic
weedDetectionPub = rospublisher('/parc_robot/weed_detection', 'std_msgs/String');

% Create subscribers for lidar scan and camera topics
lidarScanSub = rossubscriber('/scan','sensor_msgs/LaserScan');
rightCameraSub = rossubscriber('/right_camera/image_raw','sensor_msgs/Image');
leftCameraSub = rossubscriber('/left_camera/image_raw','sensor_msgs/Image');
zedCameraSub = rossubscriber('/camera/image_raw','sensor_msgs/Image');

% Initialize variables
robotPosition = [0; 0]; % Initialize with default position
robotOrientation = 0; % Initialize with default orientation
weed_found = [];

% Define the color threshold values
COLOR_MIN = [20/360, 159/255, 43/255];
COLOR_MAX = [37/360, 255/255, 217/255];

% ROS loop
while true
    % Receive robot status message
    robotStatusMsg = receive(robotStatusSub);
    robotStatus = robotStatusMsg.Data;
    disp(robotStatus);

    if strcmp(robotStatus, 'started')
        % Create a subscriber for GPS fix topic
        gpsFixSub = rossubscriber('/gps/fix', @gpsFixCallback);

        % Capture lidar scan
        lidarScanMsg = receive(lidarScanSub, 2);
        lidarScan = processLidarScan(lidarScanMsg);
        
        % Capture right camera image
        rightCameraMsg = receive(rightCameraSub, 2);
        rightCameraImage = processCameraImage(rightCameraMsg);
        
        % Capture left camera image
        leftCameraMsg = receive(leftCameraSub, 2);
        leftCameraImage = processCameraImage(leftCameraMsg);
        
        % Capture ZED 2i camera image
        zedCameraMsg = receive(zedCameraSub, 2);
        zedCameraImage = processCameraImage(zedCameraMsg);
        
        % Process images into HSV and create threshold
        rightCameraHSV = rgb2hsv(reshape(rightCameraImage, [480, 360, 3]));
        leftCameraHSV = rgb2hsv(reshape(leftCameraImage, [480, 360, 3]));
        zedCameraHSV = rgb2hsv(reshape(zedCameraImage, [480, 360, 3]));
        
        rightCameraThreshold = createThreshold(rightCameraHSV);
        leftCameraThreshold = createThreshold(leftCameraHSV);
        zedCameraThreshold = createThreshold(zedCameraHSV);
        
        % Find blob cartesian coordinates and add to weed_found array
        rightCameraBlob = findBlobCoordinates(rightCameraThreshold);
        leftCameraBlob = findBlobCoordinates(leftCameraThreshold);
        zedCameraBlob = findBlobCoordinates(zedCameraThreshold);
        
        % Transform blob coordinates to global frame
        rightCameraBlobGlobal = transformCoordinates(rightCameraBlob, robotPosition, robotOrientation);
        leftCameraBlobGlobal = transformCoordinates(leftCameraBlob, robotPosition, robotOrientation);
        zedCameraBlobGlobal = transformCoordinates(zedCameraBlob, robotPosition, robotOrientation);
        
        % Append detected weed coordinates to the weed_found array
        weed_found = [weed_found; rightCameraBlobGlobal; leftCameraBlobGlobal; zedCameraBlobGlobal];
        
    elseif strcmp(robotStatus, 'finished')
        if ~isempty(weed_found)
            % Convert weed_found to JSON string
            jsonStr = jsonencode(weed_found);
            disp(jsonStr);
            % Create a message with the JSON string
            weedDetectionMsg = rosmessage('std_msgs/String');
            weedDetectionMsg.Data = jsonStr;
            
            % Publish weed_detection message
            send(weedDetectionPub, weedDetectionMsg);
        else
            disp('No weed detected.');
        end
        
        % Exit the loop
        break;
    end
end

% ROS shutdown
rosshutdown;

% Function to process lidar scan
function lidarScanData = processLidarScan(lidarScanMsg)
    % Extract the lidar scan data from the message
    lidarScanData = lidarScanMsg.Ranges; 
end

% Function to process camera image
function cameraImage = processCameraImage(cameraMsg)
    % Extract the camera image data from the message
    cameraImage = cameraMsg.Data; 
end

% Function to create threshold based on HSV values
function thresholdImage = createThreshold(imageHSV)
    % Create a logical mask based on the HSV threshold values
    mask = (imageHSV(:, :, 1) >= COLOR_MIN(1)) & (imageHSV(:, :, 1) <= COLOR_MAX(1)) & ...
           (imageHSV(:, :, 2) >= COLOR_MIN(2)) & (imageHSV(:, :, 2) <= COLOR_MAX(2)) & ...
           (imageHSV(:, :, 3) >= COLOR_MIN(3)) & (imageHSV(:, :, 3) <= COLOR_MAX(3));
       
    % Convert the logical mask to uint8
    thresholdImage = uint8(mask);
end

% Function to find blob cartesian coordinates
function blobCoordinates = findBlobCoordinates(thresholdImage)
    % Find connected components in the threshold image
    cc = bwconncomp(thresholdImage);
    
    % Get the centroid of each connected component
    s = regionprops(cc, 'Centroid');
    centroids = cat(1, s.Centroid);
    
    % Convert centroid coordinates to [x, y] format
    blobCoordinates = fliplr(centroids);
end

% Function to transform coordinates from robot frame to global frame
function transformedCoordinates = transformCoordinates(coordinates, robotPosition, robotOrientation)
    % Rotation matrix for the robot's orientation
    rotationMatrix = [cos(robotOrientation), -sin(robotOrientation); sin(robotOrientation), cos(robotOrientation)];
    
    % Apply the transformation to the coordinates
    transformedCoordinates = (coordinates * rotationMatrix) + robotPosition;
end

% Callback function for GPS data
function gpsFixCallback(~, gpsFixMsg)
    % Declare the robotPosition variable as global
    global robotPosition;
    % Extract the robot position from the GPS fix message
    robotPosition = [gpsFixMsg.Longitude; gpsFixMsg.Latitude];
end
