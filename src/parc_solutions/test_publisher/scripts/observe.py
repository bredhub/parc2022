#!/usr/bin/env python3
## Install the geographiclib 2.0 module for this code to work.
## To install geographiclib 2.0, copy the line below to your terminal.
## pip install geographiclib
## Any of the PARC competition task must be running for this code to work.

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, LaserScan, Image,Imu
from parc_robot.gps2cartesian import gps_to_cartesian
import time
import math
import cv2
from cv_bridge import CvBridge
import numpy as np
import pyvista as pv

def calc_distance(coordA, coordB):
    pointA = [6.471, -2.124]
    pointB = [coordA, coordB]
    return math.dist(pointA, pointB)


ros_topics = {
    "gps/fix":{
        "data":None,
        "type":NavSatFix
    },
    "scan":{
        "data":None,
        "type":LaserScan
    },
    "left_camera/image_raw":{
        "data":None,
        "type":Image
    },
    "right_camera/image_raw":{
        "data":None,
        "type":Image
    },
    "zed2/imu/data":{
        "data":None,
        "type":Imu
    },
    
}

def convert_sdf_to_image(sdf_file, image_file):
    # Load the .sdf model
    mesh = pv.read(sdf_file)

    # Plot the model
    plotter = pv.Plotter(off_screen=True)
    plotter.add_mesh(mesh)
    plotter.show(screenshot=image_file)
    

def delete_converted_image(image_file):
    import os
    os.remove(image_file)
    
    
def compare_images(image_file1, image_file2):
    # Load the first image
    image1 = cv2.imread(image_file1)

    # Convert the second .sdf image to an image file
    image_file2_converted = "image2_converted.jpg"
    convert_sdf_to_image(image_file2, image_file2_converted)

    # Load the converted image
    image2 = cv2.imread(image_file2_converted)

    # Calculate the MSE
    mse = np.mean((image1 - image2) ** 2)

    # Delete the converted image file
    delete_converted_image(image_file2_converted)

    # Return the similarity score
    similarity_score = 1 / mse
    return similarity_score

  

def stop_robot():
    # Create a Twist message with zero linear and angular velocities
    stop_cmd = Twist()
    stop_cmd.linear.x = 0.0
    stop_cmd.angular.z = 0.0

    # Publish the stop command to the '/cmd_vel' topic
    pub.publish(stop_cmd)
    


def calculate_distance(latitude, longitude):
    # Implement your calculation logic here
    # You can use the GPS coordinates and any other relevant information to calculate the distance and angle to the image

    # Example calculation:
    # Assume the robot's position is at (0, 0)
    robot_latitude = 0.0
    robot_longitude = 0.0

    # Calculate the distance between the robot and the image
    dlat = math.radians(latitude - robot_latitude)
    dlon = math.radians(longitude - robot_longitude)
    a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(latitude)) * math.cos(math.radians(robot_latitude)) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = 6371e3 * c  # Distance in meters

    # Calculate the angle between the robot's heading and the image
    dx = longitude - robot_longitude
    dy = latitude - robot_latitude
    angle = math.degrees(math.atan2(dy, dx))

    return distance, angle
    
    
def continue_robot_movement():
    # Create a Twist message with desired linear and angular velocities
    move_cmd = Twist()
    move_cmd.linear.x = 0.5  # Adjust the linear velocity as desired
    move_cmd.angular.z = 0.0  # Adjust the angular velocity as desired

    # Publish the movement command to the '/cmd_vel' topic
    pub.publish(move_cmd)
    
def move_robot(data,args):
    
    
    
    # zed2_data = args[0]topic_name
    # scanner = args[1]
    topic_data = ros_topics[args]["data"]
    topic_data = data
    
    gps = None
    
    if args == "gps/fix":
        gps = topic_data
        # print(gps.latitude)
        latitude = gps.latitude
        longitude = gps.longitude

        # Calculate the distance and angle to the image
        distance, angle = calculate_distance(latitude, longitude)
        

        # Print the distance and angle
        print("Distance to image:", distance)
        print("Angle to image:", angle)
        
    if args == "scan":
        scan_data = data
        # print(scan_data)
        # Access the laser scan data
        ranges = scan_data.ranges  # List of range measurements
        min_range = min(ranges)  # Minimum range value
        # print(min_range)
        # Set the desired minimum safe distance to avoid obstacles
        safe_distance = 0.17  # Adjust this value according to your requirements

        if min_range < safe_distance:
            # Obstacle detected, stop the robot
            stop_robot()
        else:
            # No obstacle detected, continue with robot movement
            continue_robot_movement()
        if args == "left_camera/image_raw":
            image_data = topic_data

            # Perform image processing and object detection to identify the image
            # Here, we assume the image is already converted to OpenCV format (BGR)
            # Implement your image processing and object detection algorithms
            # ...

            # Example code:
            # Convert the image to grayscale
            gray_image = cv2.cvtColor(image_data, cv2.COLOR_BGR2GRAY)

            # Apply a threshold to separate foreground and background
            _, thresholded_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)

            # Find contours in the thresholded image
            contours, _ = cv2.findContours(thresholded_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Assume the image is identified if any contours are found
            if len(contours) > 0:
                distance = calculate_distance(image_data)
                # Image detected, stop the robot
                if distance <= 0.2:
                # Robot is within 0.2 meters of the image, stop the robot
                    stop_robot()
                else:
                    # Robot is farther than 0.2 meters from the image, continue with robot movement
                    continue_robot_movement()
            else:
                # No image detected, continue with robot movement
                continue_robot_movement()
                
    
    
    

    rate = rospy.Rate(10)

    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()
if __name__ == '__main__':
    try:
        lettuce = convert_sdf_to_image("~/catkin_ws/src/PARC-Engineers-League/parc_robot/models/lettuce_crop_02/lettuce_crop_02.sdf", "lettuce.jpg")
        rospy.init_node('robot_publisher', anonymous=True)
        # Create a publisher which can "talk" to Robot and tell it to move
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        time.sleep(1)

        # Start the robot's movement
        continue_robot_movement()
        # sub = rospy.Subscriber('gps/fix', NavSatFix, move_robot)
        for key in ros_topics.keys():
            rospy.Subscriber(name=key, data_class= ros_topics[key]["type"], callback=move_robot, callback_args=key)
        # move_robot()rate = rospy.Rate(10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    

