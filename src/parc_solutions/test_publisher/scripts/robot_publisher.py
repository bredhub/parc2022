#!/usr/bin/env python3
## Install the geographiclib 2.0 module for this code to work.
## To install geographiclib 2.0, copy the line below to your terminal.
## pip install geographiclib
## Any of the PARC competition task must be running for this code to work.
import rospy
from sensor_msgs.msg import LaserScan, Image, NavSatFix
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from parc_robot.gps2cartesian import gps_to_cartesian
from cv_bridge import CvBridge
import cv2
import math
import random

YOUR_PROXIMITY_THRESHOLD = 0.01

class RobotController:
    def __init__(self):
        rospy.init_node('collision_avoidance_node')
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/left_camera/image_raw', Image, self.left_camera_callback)
        rospy.Subscriber('/right_camera/image_raw', Image, self.right_camera_callback)
        rospy.Subscriber('/gps', NavSatFix, self.gps_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.models = []
        self.robot_twist = Twist()
        self.collision_distance = 0.012
        self.bridge = CvBridge()
        self.model_positions = {}  # Dictionary to store the positions of models
        self.goal_reached = False
        self.desired_linear_vel = 0.0
        self.desired_angular_vel = 0.0

    def lidar_callback(self, scan_msg):
        min_range = min(scan_msg.ranges)
        if min_range < self.collision_distance:
            # Collision detected, stop the robot or perform evasive action
            self.desired_linear_vel = 0.0
            self.desired_angular_vel = 0.0
        else:
            # No collision detected, continue moving forward with desired velocities
            self.desired_linear_vel = 0.2  # Set the desired linear velocity
            self.desired_angular_vel = 0.0

    def left_camera_callback(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # Implement collision avoidance logic using the left camera image and the models

        # Example: Detecting objects using color-based segmentation
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_green = (40, 40, 40)
        upper_green = (70, 255, 255)
        mask = cv2.inRange(hsv_image, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # Get the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)
            # Check if the bounding box intersects with any of the model regions
            if self.check_collision_with_models(x, y, w, h):
                # Collision detected, perform evasive action
                self.desired_linear_vel = 0.0
                self.desired_angular_vel = random.uniform(-1.0, 1.0)  # Randomly drift left or right
                break
        else:
            # No collision detected, continue moving forward with desired velocities
            self.desired_linear_vel = 0.2  # Set the desired linear velocity
            self.desired_angular_vel = 0.0

    def right_camera_callback(self, image_msg):
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # Implement obstacle avoidance logic using the right camera image

        # Example: Detecting obstacles using color-based segmentation
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_obstacle = (0, 0, 0)
        upper_obstacle = (10, 255, 255)
        mask = cv2.inRange(hsv_image, lower_obstacle, upper_obstacle)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Iterate through detected contours
        for contour in contours:
            # Get the bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)
            
            if self.check_collision_with_models(x, y, w, h):
                self.desired_linear_vel = 0.0
                self.desired_angular_vel = random.uniform(-1.0, -0.5)  # Randomly drift left
                break
            
            # Check if the bounding box is within a specific range on the right camera image
            elif x < (cv_image.shape[1] // 2) and x + w > (cv_image.shape[1] // 2 - 100):
                # Collision detected on the right side, perform evasive action
                self.desired_linear_vel = 0.0
                self.desired_angular_vel = random.uniform(-1.0, -0.5)  # Randomly drift left
                break
        else:
            # No collision detected, continue moving forward with desired velocities
            self.desired_linear_vel = 0.2  # Set the desired linear velocity
            self.desired_angular_vel = 0.0

    def gps_callback(self, gps_msg):
        # Process GPS data for localization or navigation purposes
        # Update robot's position or use it in decision-making algorithms
        
        # Example: Get the latitude and longitude from the GPS message
        latitude = gps_msg.latitude
        longitude = gps_msg.longitude
        
        # Convert GPS coordinates to Cartesian coordinates for distance calculation
        robot_x, robot_y = gps_to_cartesian(latitude, longitude)
        
        # Get the position of the goal_location model
        goal_x = self.model_positions['goal_location'].position.x
        goal_y = self.model_positions['goal_location'].position.y
        
        # Calculate the Euclidean distance between the robot's current position and the goal location
        distance = math.sqrt((robot_x - goal_x)**2 + (robot_y - goal_y)**2)
    
        if distance < YOUR_PROXIMITY_THRESHOLD:
            # Robot is in proximity to the goal location, stop the robot or perform desired action
            self.desired_linear_vel = 0.0
            self.desired_angular_vel = 0.0

    def model_states_callback(self, model_states_msg):
        # Update the model positions dictionary
        model_names = model_states_msg.name
        model_poses = model_states_msg.pose

        for name, pose in zip(model_names, model_poses):
            self.model_positions[name] = pose
            self.models.append(name)

        # print(self.model_positions)
        # print(self.models)

        # Check if the robot has reached the goal location
        if 'goal_location' in self.model_positions and not self.goal_reached:
            goal_z = self.model_positions['goal_location'].position.z
            if goal_z < 0.05:  # Adjust the threshold based on the goal's height
                self.goal_reached = True
                # Robot has reached the goal location, stop the robot or perform desired action
                self.desired_linear_vel = 0.0
                self.desired_angular_vel = 0.0

    def check_collision_with_models(self, x, y, w, h):
        for model_name in self.models:
            if model_name.startswith("lettuce_"):
                model_pose = self.model_positions[model_name]
                model_x = model_pose.position.x
                model_y = model_pose.position.y
                model_w = 0.2 # Provide the width of the model bounding box
                model_h = 0.2 # Provide the height of the model bounding box

                if (x < model_x + model_w and
                        x + w > model_x and
                        y < model_y + model_h and
                        y + h > model_y):
                    return True
        return False

    def control_loop(self):
        rate = rospy.Rate(10)  # 10 Hz control loop
        while not rospy.is_shutdown():
            self.robot_twist.linear.x = self.desired_linear_vel
            self.robot_twist.angular.z = self.desired_angular_vel
            self.pub_cmd_vel.publish(self.robot_twist)
            rate.sleep()


if __name__ == '__main__':
    controller = RobotController()
    try:
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
