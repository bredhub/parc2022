% Install the geographiclib 2.0 module for this code to work.
% Any of the PARC competition task must be running for this code to work.

% Add necessary ROS packages to MATLAB's environment
setenv('ROS_MASTER_URI', 'http://localhost:11311')
setenv('ROS_IP', 'localhost')

% Initialize ROS node
rosinit('NodeName', 'robot_publisher')

% Create a publisher to send movement commands to the robot
pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
move_cmd = rosmessage(pub);

% Initialize the robot's movement
move_cmd.Linear.X = 0.5;  % Adjust the linear velocity as desired
move_cmd.Angular.Z = 0.0;  % Adjust the angular velocity as desired
send(pub, move_cmd);

% Define the ROS topics and their corresponding data types
ros_topics = struct();
ros_topics('gps/fix') = struct('data', [], 'type', 'sensor_msgs/NavSatFix');
ros_topics('scan') = struct('data', [], 'type', 'sensor_msgs/LaserScan');
ros_topics('left_camera/image_raw') = struct('data', [], 'type', 'sensor_msgs/Image');
ros_topics('right_camera/image_raw') = struct('data', [], 'type', 'sensor_msgs/Image');
ros_topics('zed2/imu/data') = struct('data', [], 'type', 'sensor_msgs/Imu');

% Define the safe distance to avoid obstacles
safe_distance = 0.17;  % Adjust this value according to your requirements

% Define the distance threshold to detect the image
image_distance_threshold = 0.2;  % Adjust this value according to your requirements

% Define the coordinates of the reference point A
pointA = [6.471, -2.124];

% Define the image processing and object detection algorithms
% Implement your own logic here

% Define the callback function for each subscribed topic
for key = keys(ros_topics)
    % Extract the topic name and data type
    topic_name = key{1};
    topic_data_type = ros_topics(topic_name).type;
    
    % Create a subscriber for the current topic
    sub = rossubscriber(topic_name, topic_data_type, @(src, msg) move_robot(src, msg, topic_name));
end

% Callback function to process the received data and control the robot's movement
function move_robot(~, msg, topic_name)
    global ros_topics pointA move_cmd pub safe_distance image_distance_threshold
    
    switch topic_name
        case 'gps/fix'
            % Access the GPS fix data
            gps = msg;
            
            % Extract the latitude and longitude values
            latitude = gps.Latitude;
            longitude = gps.Longitude;
            
            % Calculate the distance and angle to the image
            [distance, angle] = calculate_distance(latitude, longitude);
            
            % Print the distance and angle
            fprintf('Distance to image: %.2f\n', distance);
            fprintf('Angle to image: %.2f\n', angle);
            
        case 'scan'
            % Access the laser scan data
            ranges = msg.Ranges;  % List of range measurements
            min_range = min(ranges);  % Minimum range value
            
            if min_range < safe_distance
                % Obstacle detected, stop the robot
                stop_robot();
            else
                % No obstacle detected, continue with robot movement
                continue_robot_movement();
            end
            
        case {'left_camera/image_raw', 'right_camera/image_raw'}
            % Access the image data
            image_data = readImage(msg);
            
            % Perform image processing and object detection to identify the image
            % Implement your image processing and object detection algorithms
            % ...
            
            % Example code:
            % Convert the image to grayscale
            gray_image = rgb2gray(image_data);
            
            % Apply a threshold to separate foreground and background
            thresholded_image = gray_image > 127;
            
            % Find contours in the thresholded image
            contours = bwboundaries(thresholded_image);
            
            % Assume the image is identified if any contours are found
            if ~isempty(contours)
                [distance, ~] = calculate_distance(image_data);
                
                if distance <= image_distance_threshold
                    % Robot is within the desired distance to the image, stop the robot
                    stop_robot();
                else
                    % Robot is farther than the desired distance to the image, continue with robot movement
                    continue_robot_movement();
                end
            else
                % No image detected, continue with robot movement
                continue_robot_movement();
            end
    end
end

% Function to calculate the distance and angle between the robot's position and an image
function [distance, angle] = calculate_distance(latitude, longitude)
    global pointA
    
    % Assume the robot's position is at (0, 0)
    robot_latitude = 0.0;
    robot_longitude = 0.0;
    
    % Calculate the distance between the robot and the image
    dlat = deg2rad(latitude - robot_latitude);
    dlon = deg2rad(longitude - robot_longitude);
    a = sin(dlat/2)^2 + cos(deg2rad(latitude)) * cos(deg2rad(robot_latitude)) * sin(dlon/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));
    distance = 6371e3 * c;  % Distance in meters
    
    % Calculate the angle between the robot's heading and the image
    dx = longitude - robot_longitude;
    dy = latitude - robot_latitude;
    angle = rad2deg(atan2(dy, dx));
end

% Function to stop the robot's movement
function stop_robot()
    global move_cmd pub
    
    % Set the linear and angular velocities to zero
    move_cmd.Linear.X = 0.0;
    move_cmd.Angular.Z = 0.0;
    
    % Publish the stop command
    send(pub, move_cmd);
end

% Function to continue the robot's movement
function continue_robot_movement()
    global move_cmd pub
    
    % Set the linear velocity as desired and angular velocity to zero
    move_cmd.Linear.X = 0.5;  % Adjust the linear velocity as desired
    move_cmd.Angular.Z = 0.0;
    
    % Publish the movement command
    send(pub, move_cmd);
end

% Function to calculate the Euclidean distance between two points
function distance = calc_distance(coordA, coordB)
    pointB = [coordA, coordB];
    distance = norm(pointA - pointB);
end

% Main script
try
    % Wait for the specified duration before starting the robot's movement
    pause(1);
    
    % Start the robot's movement
    continue_robot_movement();
    
    % Keep MATLAB running until interrupted
    disp("Press Ctrl+C to stop the script...");
    while true
        pause(0.1);
    end
    
catch ex
    disp("An error occurred:");
    disp(ex.message);
    
    % Stop the robot in case of an error
    stop_robot();
end
