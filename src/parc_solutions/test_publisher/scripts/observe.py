#!/usr/bin/env python3
## Install the geographiclib 2.0 module for this code to work.
## To install geographiclib 2.0, copy the line below to your terminal.
## pip install geographiclib
## Any of the PARC competition task must be running for this code to work.
   

import rospy
from sensor_msgs.msg import LaserScan, Image, NavSatFix, CameraInfo, PointCloud2
from geometry_msgs.msg import Twist, Pose
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from parc_robot.gps2cartesian import gps_to_cartesian
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion
import cv2
import math
import numpy as np

YOUR_PROXIMITY_THRESHOLD = 0.1594
SIDE_PROX_LOW = 0.13
SIDE_PROX_HIGH = 2.0
SIDE_PROX_HIGH_ = 2.0
IMAGE_RANGE = 19.50
LESS_RANGE = 19.20

class RobotController:
    def __init__(self):
        rospy.init_node('collision_avoidance_node')
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/left_camera/image_raw', Image, self.left_camera_callback)
        rospy.Subscriber('/right_camera/image_raw', Image, self.right_camera_callback)
        rospy.Subscriber('/right_camera/camera_info', CameraInfo, self.camera_info_right_callback)
        rospy.Subscriber('/left_camera/camera_info', CameraInfo, self.camera_info_left_callback)
        rospy.Subscriber("/zed2/point_cloud/cloud_registered", PointCloud2, self.point_cloud_callback)
        rospy.Subscriber('odom', Odometry , self.odometry_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_twist = Twist()
        self.collision_distance = YOUR_PROXIMITY_THRESHOLD
        self.bridge = CvBridge()
        self.goal_reached = False
        self.desired_linear_vel = 0.1
        self.desired_angular_vel = 0.0
        self.left_camera_info = None
        self.right_camera_info = None
        self.robot_pose = 0.0
        self.left = False
        self.right = False
        self.is_drift = False
        # Register the coordinates of the pegs
        peg_01 = rospy.get_param('peg_01')
        peg_02 = rospy.get_param('peg_02')
        peg_03 = rospy.get_param('peg_03')
        peg_04 = rospy.get_param('peg_04')
        peg_05 = rospy.get_param('peg_05')
        peg_06 = rospy.get_param('peg_06')
        peg_07 = rospy.get_param('peg_07')
        peg_08 = rospy.get_param('peg_08')
       

        self.peg_01_lat, self.peg_01_lon = peg_01['latitude'], peg_01['longitude']
        self.peg_02_lat, self.peg_02_lon = peg_02['latitude'], peg_02['longitude']
        self.peg_03_lat, self.peg_03_lon = peg_03['latitude'], peg_03['longitude']
        self.peg_04_lat, self.peg_04_lon = peg_04['latitude'], peg_04['longitude']
        self.peg_05_lat, self.peg_05_lon = peg_05['latitude'], peg_05['longitude']
        self.peg_06_lat, self.peg_06_lon = peg_06['latitude'], peg_06['longitude']
        self.peg_07_lat, self.peg_07_lon = peg_07['latitude'], peg_07['longitude']
        self.peg_08_lat, self.peg_08_lon = peg_08['latitude'], peg_08['longitude']
        
        self.point1_gps = self.mean_gps(self.peg_01_lat, self.peg_01_lon, self.peg_04_lat, self.peg_04_lon)
        self.point2_gps = self.mean_gps(self.peg_02_lat, self.peg_02_lon, self.peg_03_lat, self.peg_03_lon)
        self.point3_gps = self.mean_gps(self.peg_03_lat, self.peg_03_lon,self.peg_06_lat, self.peg_06_lon)
        self.point4_gps = self.mean_gps(self.peg_04_lat, self.peg_04_lon,self.peg_05_lat, self.peg_05_lon)
        self.point5_gps = self.mean_gps(self.peg_05_lat, self.peg_05_lon,self.peg_08_lat, self.peg_08_lon)
        self.point6_gps = self.mean_gps(self.peg_07_lat, self.peg_07_lon,self.peg_06_lat, self.peg_06_lon)
        #gps to cartesian
        self.point1 = self.mean_average(gps_to_cartesian(self.peg_01_lat, self.peg_01_lon), gps_to_cartesian(self.peg_04_lat, self.peg_04_lon))
        self.point2 = self.mean_average(gps_to_cartesian(self.peg_02_lat, self.peg_02_lon), gps_to_cartesian(self.peg_03_lat, self.peg_03_lon))
        self.point3 = self.mean_average(gps_to_cartesian(self.peg_03_lat, self.peg_03_lon), gps_to_cartesian(self.peg_06_lat, self.peg_06_lon))
        self.point4 = self.mean_average(gps_to_cartesian(self.peg_04_lat, self.peg_04_lon), gps_to_cartesian(self.peg_05_lat, self.peg_05_lon))
        self.point5 = self.mean_average(gps_to_cartesian(self.peg_05_lat, self.peg_05_lon), gps_to_cartesian(self.peg_08_lat, self.peg_08_lon))
        self.point6 = self.mean_average(gps_to_cartesian(self.peg_07_lat, self.peg_07_lon), gps_to_cartesian(self.peg_06_lat, self.peg_06_lon))
        
        self.current_goal_met = False
        self.current_goal = self.point2 
        self.current_gps_goal = self.point2_gps
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.image_height = 0  
        self.left_angle = False
        self.right_angle = False 
        
    
    def point_cloud_callback(self, data):
        min_height = float('inf')
        max_height = float('-inf')

        # Iterate over each point in the point cloud
        for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point

            # Update the minimum and maximum height
            min_height = min(min_height, z)
            max_height = max(max_height, z)

        height_range = max_height - min_height
        self.image_height = height_range
        print("Height range:", height_range) 
        # if self.image_height >  IMAGE_RANGE and not self.is_drift :
        #     print("icloud")
        #     self.drift()
        
    def camera_info_callback(self,data):
        self.camera_matrix = np.array(data.K).reshape(3, 3)
        self.distortion_coeffs = np.array(data.D)
        
    def image_callback(self, data):
        print("imu")
        # Convert the ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Analyze the height of the image
        height, width, _ = cv_image.shape
        print("Image height:", height)
        
    def camera_info_right_callback(self, camera_info_msg):
        # Store the camera information for calibration
        self.right_camera_info = camera_info_msg
        
        
    def camera_info_left_callback(self, camera_info_msg):
        # Store the camera information for calibration
        self.left_camera_info = camera_info_msg
    
    
    def depth_callback(self, depth_msg):
        try:
           # Convert the depth image to a numpy array
            if self.camera_matrix is None or self.distortion_coeffs is None:
                rospy.logwarn("Camera info not received yet")
                return
            
            # Convert the depth image to a numpy array
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            
            # Apply camera calibration parameters to the depth image
            depth_image_rectified = cv2.undistort(depth_image, self.camera_matrix, self.distortion_coeffs)
            
            # Normalize the depth values
            depth_image_rectified = depth_image_rectified / 1000.0  # Convert from millimeters to meters

            # Analyze the height of the image based on the depth values
            min_depth = np.min(depth_image_rectified)
            max_depth = np.max(depth_image_rectified)
            height_range = max_depth - min_depth
            print("Height range:", height_range)
                   

        except Exception as e:
            rospy.logerr(e)

    
    def get_distance(cv_image, x, y):
        # Extract depth information from the image (assuming depth values are in meters)
        depth_image = cv_image[:, :, 2]  # Assuming depth is stored in the blue channel
        distance = depth_image[y, x]
        return distance

    
    def mean_gps(self, x1,y1, x2, y2):
        # Convert degrees to radians
        lat1 = math.radians(x1)
        lon1 = math.radians(y1)
        lat2 = math.radians(x2)
        lon2 = math.radians(y2)

        # Find the average of the latitudes and the longitudes
        avg_lat = (lat1 + lat2) / 2
        avg_lon = (lon1 + lon2) / 2

        # Convert back from radians to degrees
        midpoint_latitude = math.degrees(avg_lat)
        midpoint_longitude = math.degrees(avg_lon)

        return [midpoint_latitude, midpoint_longitude]
        
    def mean_average(self, pointA, pointB):
        x1, y1 = pointA
        x2, y2 = pointB
        mid_x = (x1+x2)/2
        mid_y = (y1 + y2)/ 2
        return [mid_x, mid_y]
    
    
    def odometry_callback(self, odom_msg):
        # Extract the robot's orientation from the odometry message
        orientation = odom_msg.pose.pose.orientation
        
        # Convert the orientation to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        
        # Update the robot's current orientation (theta)
        self.robot_pose = yaw
    
    
    def drift_right(self):
        # Rotate in the opposite direction
        self.desired_linear_vel = 0.0
        if self.robot_pose < 0  and not self.right_angle: 
        # if self.left:
            # Robot is facing forward or towards the right, rotate right
            print("kindly drift right 2")
            self.desired_angular_vel = -0.04
        else:
            # Robot is facing towards the left, rotate left
            print("kindly drift left 2")
            self.desired_angular_vel = 0.04
        self.publish_twist()
        rospy.sleep(1)
        self.left_angle = False
        self.right_angle = True
        
           
            
            
        
    def drift_left(self):
        
        self.desired_linear_vel = 0.0
        if self.robot_pose > 0 :
        # if self.right:
            # Robot is facing forward or towards the right, drift left
            self.desired_angular_vel = 0.04
            print("kindly drift left")
        else:
            # Robot is facing towards the left, drift right
            self.desired_angular_vel = -0.04
            print("kindly druift right")
        self.publish_twist()
        
        # Move away from the obstacle for a certain duration
        rospy.sleep(1)  # Adjust the duration as needed
        
        
        # self.desired_linear_vel = 0.0
        # self.desired_angular_vel = 0.0
        # self.publish_twist()
        # self.left_angle = True
        # self.right_angle = False
    
    def drift(self):
        print(self.robot_pose)
        self.is_drift = True
        self.desired_linear_vel = 0.0
        if self.current_goal_met == True:
            self.desired_angular_vel = 0.0
            self.publish_twist()
            rospy.sleep(5)
        else:
            if self.left:
                print("in n")
                self.desired_linear_vel = 0.0
                self.desired_angular_vel = -0.04
                self.publish_twist()
                rospy.sleep(1)
            elif self.right:
                print("in t")
                self.desired_linear_vel = 0.0
                self.desired_angular_vel = 0.04
                self.publish_twist()
                rospy.sleep(1)
            elif self.robot_pose > 0 and not (self.image_height < IMAGE_RANGE  and self.image_height > LESS_RANGE)  :
                print("drift lett")
                self.drift_left()
            elif self.robot_pose < 0 and not (self.image_height  < IMAGE_RANGE and self.image_height > LESS_RANGE):
                print("dirft right")
                self.drift_right()
                
            else:
                self.desired_linear_vel = 0.0
                self.desired_angular_vel = 0.0
                self.publish_twist()
                rospy.sleep(5)
        
        self.is_drift = False
    def lidar_callback(self, scan_msg):
        min_range = min(scan_msg.ranges)
        
       
       
        
        if min_range < YOUR_PROXIMITY_THRESHOLD:
            if not (self.image_height < IMAGE_RANGE  and self.image_height > LESS_RANGE) and not self.is_drift :
                # Collisited, perform evasive action
                self.right = False
                self.left = False
                print("lidar drift")
                self.drift()
   
            
        else: 
            if not self.current_goal_met:
                # No collision detected, continue moving forward with desired velocities
                self.desired_linear_vel = 0.2  # Set the desired linear velocity
                self.desired_angular_vel = 0.0
                self.publish_twist()

    def left_camera_callback(self, image_msg):
        
        if self.left_camera_info is None:
            return
       
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        # Add your image processing code here
        # Example: Convert the image to grayscale
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_obstacle = (0, 0, 0)
        upper_obstacle = (10, 255, 255)
        mask = cv2.inRange(hsv_image, lower_obstacle, upper_obstacle)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through detected contours
        for contour in contours:
            # Get the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)
            # Check if the bounding box intersects with any of the model regions
            if x < (cv_image.shape[1] // 2) and x + w > (cv_image.shape[1] // 2 - 100):
                # Calculate the distance to the obstacle using the width of the bounding box
                distance_to_obstacle = self.calculate_distance_to_obstacle(w, "right")
              
                print("left" + str(distance_to_obstacle))
                # Check if the robot is too close to the obstacle
                if distance_to_obstacle < SIDE_PROX_LOW and not self.is_drift :
                    self.right = False
                    self.left = True
                    self.drift()
                elif distance_to_obstacle > SIDE_PROX_HIGH and not self.is_drift :
                    self.right = False
                    self.left = True
                    self.drift()
                    # self.drift()

            
    def calculate_distance_to_obstacle(self, width, side):
        # Assuming a fixed relation between width and distance (example values)
        # Adjust these values based on your specific setup and measurements
        obstacle_width = 0.1  # Width of the obstacle in meters
        focal_length = 500.0  # Focal length of the camera in pixels
        
        if side == "right" and self.right_camera_info is not None:
            
            K = np.array(self.right_camera_info.K).reshape(3, 3)
            focal_length = K[0, 0]
            
        elif side =="left" and self.left_camera_info is not None:
            K = np.array(self.left_camera_info.K).reshape(3, 3)
            focal_length = K[0, 0]
           
        else:
            focal_length = 0
        # Calculate the distance using the formula: distance = (object_width * focal_length) / width
        distance = (obstacle_width * focal_length) / width
        
        return distance

    def right_camera_callback(self, image_msg):
       
        if self.right_camera_info is None:
            return
        
        # Process the right camera image
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        
        
        # Convert the image to HSV format for color thresholding
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_obstacle = (0, 0, 0)
        upper_obstacle = (10, 255, 255)
        mask = cv2.inRange(hsv_image, lower_obstacle, upper_obstacle)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Iterate through detected contours
        for contour in contours:
            # Get the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Check if the bounding box is within a specific range on the right camera image
            if x < (cv_image.shape[1] // 2) and x + w > (cv_image.shape[1] // 2 - 100):
                # Calculate the distance to the obstacle using the width of the bounding box
                distance_to_obstacle = self.calculate_distance_to_obstacle(w, "right")
                # print("right" + str(distance_to_obstacle))
                # Check if the robot is too close to the obstacle
                if distance_to_obstacle < SIDE_PROX_LOW  and not self.is_drift:
                    self.right = True
                    self.left = False
                    self.drift()
                elif distance_to_obstacle > SIDE_PROX_HIGH and not self.is_drift :
                    self.right = True
                    self.left = False
                    self.drift()
                    # print(distance_to_obstacle)
                    # print("right")
                    # Collision detected, stop the robot
                    # self.right = True
                    # self.left = False
                    # self.drift()
                    
       


    def gps_callback(self, gps_msg):
       
        # Get the current GPS coordinates
        
        current_lat = gps_msg.latitude
        current_lon = gps_msg.longitude
        
    
        x, y = gps_to_cartesian(current_lat, current_lon)
        # Check if the goal has been reached
        print(x, y)
        if self.check_goal_reached(x, y):
            print("first goal met")
            
            if self.current_goal  == self.point2:
                self.current_goal = self.point3
                
                self.current_gps_goal = self.point3_gps
            elif self.current_goal == self.point3:
                self.current_goal = self.point4
                self.current_gps_goal = self.point4_gps
            elif self.current_goal == self.point4:
                self.current_goal = self.point5
                self.current_gps_goal = self.point5_gps
            elif self.current_goal == self.point5:
                self.current_goal = self.point6
                self.current_gps_goal = self.point6_gps
            elif self.current_goal == self.point6:
                self.goal_reached = True
            
            self.current_goal_met = True
            # self.drift()
            
            # self.desired_linear_vel = 0.0
            # self.desired_angular_vel = 0.0

            # Calculate the desired heading angle towards the next peg
            # desired_heading = self.calculate_desired_heading(current_lat, current_lon)
            

            # Adjust the angular velocity to steer towards the desired heading
            # self.desired_angular_vel = self.calculate_angular_velocity(desired_heading)

            # Publish the desired twist command to move the robot
            # self.publish_twist()
            # rospy.sleep(5)

    def check_goal_reached(self, current_lat, current_lon):
        # Calculate the distance between current position and the goal position
        goal_lat = self.current_goal[0]
        goal_lon = self.current_goal[1]
        distance = self.calculate_distance(current_lat, current_lon, goal_lat, goal_lon)
        # print(distance)
        # print("current")
        # Check if the distance is below the threshold
        if distance > 1 and current_lat < goal_lat and current_lon > goal_lon :
            return True
        else:
            return False
        
    def calculate_distance(self, x1, y1, x2, y2):
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return distance
        

    def calculate_desired_heading(self, current_lat, current_lon):
        # Calculate the angle between the current position and the goal position
        goal_lat = self.current_gps_goal[0]
        goal_lon = self.current_gps_goal[1]
        delta_lat = goal_lat - current_lat
        delta_lon = goal_lon - current_lon

        # Convert delta_lat and delta_lon to radians
        delta_lat_rad = math.radians(delta_lat)
        delta_lon_rad = math.radians(delta_lon)

        # Calculate the desired heading angle
        desired_heading = math.atan2(delta_lon_rad, delta_lat_rad)

        return desired_heading

    def calculate_angular_velocity(self, desired_heading):
        # Calculate the angular velocity needed to steer towards the desired heading
        angular_velocity = desired_heading

        return angular_velocity

    def publish_twist(self):
        # Create a Twist message with the desired linear and angular velocities
        self.robot_twist.linear.x = self.desired_linear_vel
        self.robot_twist.angular.z = self.desired_angular_vel

        # Publish the Twist message
        self.pub_cmd_vel.publish(self.robot_twist)

    

if __name__ == '__main__':
    robot_controller = RobotController()

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown() and not robot_controller.goal_reached:
        robot_controller.publish_twist()
        rate.sleep()
