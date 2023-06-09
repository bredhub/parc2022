
% ROS initialization
rosinit;

% Create a subscriber to /parc_robot/robot_status topic
robotStatusSub = rossubscriber('/parc_robot/robot_status', 'std_msgs/String');

% Create a publisher to /parc_robot/weed_detection topic
weedDetectionPub = rospublisher('/parc_robot/weed_detection', 'std_msgs/String');

% Define the sensor topics
lidarScanTopic = rossubscriber('/scan','sensor_msgs/LaserScan');
rightCameraTopic = rossubscriber('/right_camera/image_raw','sensor_msgs/Image');
leftCameraTopic = rossubscriber('/left_camera/image_raw','sensor_msgs/Image');
zedCameraTopic = rossubscriber('/camera/image_raw','sensor_msgs/Image');

% Initialize weed_found array
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
        % Capture lidar scan
        lidarScanMsg = receive(lidarScanTopic,1);
        lidarScan = processLidarScan(lidarScanMsg);
        
        % Capture right camera image
        rightCameraMsg = receive(rightCameraTopic, 1);
        rightCameraImage = processCameraImage(rightCameraMsg);
        
        % Capture left camera image
        leftCameraMsg = receive(leftCameraTopic, 1);
        leftCameraImage = processCameraImage(leftCameraMsg);
        
        % Capture ZED 2i camera image
        zedCameraMsg = receive(zedCameraTopic, 1);
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
        
        weed_found = [weed_found; rightCameraBlob; leftCameraBlob; zedCameraBlob];
        
    elseif strcmp(robotStatus, 'finished')
        % Convert weed_found to JSON string
        jsonStr = jsonencode(weed_found);
        disp(jsonStr);
        % Create a message with the JSON string
        weedDetectionMsg = rosmessage('std_msgs/String');
        weedDetectionMsg.Data = jsonStr;
        
        % Publish weed_detection message
        send(weedDetectionPub, weedDetectionMsg);
        
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
