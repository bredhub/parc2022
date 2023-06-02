#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from parc_robot.gps2cartesian import gps_to_cartesian
import math
import cv2
import numpy as np
import os

# Global flag to track the robot status
robot_status = "finished"
# Path to the sample weed image
sample_weed_path = "~/catkin_ws/src/PARC-Engineers-League/parc_robot/models/unknown_weed/materials/textures/unknown_weed.png"

sample_weed_path = os.path.expanduser(sample_weed_path)
# Load the sample weed image
sample_weed_image = cv2.imread(sample_weed_path, cv2.IMREAD_GRAYSCALE)

detected_list = []
weed_count = 0

def lidar_callback(data):
    # Process LiDAR data here
    if robot_status == "started":
        detected_objects = extract_detected_objects(data)
        filtered_objects = filter_weed_objects(detected_objects)
        detected_list.extend(filtered_objects)

def extract_detected_objects(data):
    detected_objects = []

    for i, distance in enumerate(data.ranges):
        if(distance < 1.0):
            angle = data.angle_min + i * data.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            detected_objects.append([x, y])

    return detected_objects

def filter_weed_objects(detected_objects):
    filtered_objects = []

    for obj in detected_objects:
        if isinstance(obj, np.ndarray):  # Check if obj is a valid numpy array
            if obj.ndim > 2:  # Check if obj is a color image
                obj_gray = cv2.cvtColor(obj, cv2.COLOR_BGR2GRAY)
            else:  # obj is already grayscale
                obj_gray = obj.copy()

            # Resize the object image to match the sample weed image size
            obj_resized = cv2.resize(obj_gray, (sample_weed_image.shape[1], sample_weed_image.shape[0]))

            # Perform template matching
            result = cv2.matchTemplate(obj_resized, sample_weed_image, cv2.TM_CCOEFF_NORMED)

            # Define a similarity threshold
            threshold = 0.8

            # Find locations where the template matches above the threshold
            locations = np.where(result >= threshold)

            # Check if the object is similar to the sample weed image
            if locations[0].size > 0:
                filtered_objects.append(obj)

    return filtered_objects

def left_camera_callback(data):
    global weed_count
    # Process left camera data here
    if robot_status == "started":
        # Convert the image message to OpenCV format
        cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape((data.height, data.width, -1))
        weed_count += 1
        # Compare detected objects with the sample weed image
        matched_objects = compare_with_sample(cv_image)

        # Append the matched object coordinates to the list
        detected_list.extend(matched_objects)

def compare_with_sample(image):
    matched_objects = []

    if image.ndim > 2:
        cv_image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        cv_image_gray = image

    # Convert the sample weed image to grayscale if needed
    if sample_weed_image.ndim > 2:
        sample_weed_image_gray = cv2.cvtColor(sample_weed_image, cv2.COLOR_BGR2GRAY)
    else:
        sample_weed_image_gray = sample_weed_image

    cv_image_gray = cv_image_gray.astype(np.uint8)
    sample_weed_image_gray = sample_weed_image_gray.astype(np.uint8)

    # Perform template matching
    result = cv2.matchTemplate(cv_image_gray, sample_weed_image_gray, cv2.TM_CCOEFF_NORMED)

    # Define a threshold for the template matching result
    threshold = 0.22

    # Find locations where the template matches above the threshold
    locations = np.where(result >= threshold)

    # Append the matched object coordinates to the list
    for loc in zip(*locations[::-1]):
        matched_objects.append(loc)

    return matched_objects

def right_camera_callback(data):
    global weed_count
    # Process right camera data here
    if robot_status == "started":
        # Convert the image message to OpenCV format
        cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape((data.height, data.width, -1))

        # Perform object detection on the image or any other processing
        weed_count += 1
        # Compare detected objects with the sample weed image
        matched_objects = compare_with_sample(cv_image)

        # Append the matched object coordinates to the list
        detected_list.extend(matched_objects)

def zed_camera_callback(data):
    # Process ZED camera data here
    if robot_status == "started":
        # Convert the image message to OpenCV format
        cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape((data.height, data.width, -1))

        # Compare the ZED camera image with the sample weed image
        matched_objects = compare_with_sample(cv_image)

        # Append the matched object coordinates to the list
        detected_list.extend(matched_objects)

def gps_callback(data):
    # Process GPS data here
    if robot_status == "started":
        x, y = gps_to_cartesian(data.latitude, data.longitude)
        detected_list[-1].append((x, y))  # Append the geolocation to the last detected object

def robot_status_callback(data):
    # Process robot status here
    global robot_status

    robot_status = data.data
    if robot_status == "started":
        rospy.loginfo("Robot status: Started")
    elif robot_status == "finished":
        rospy.loginfo("Robot status: Finished")
        rospy.signal_shutdown("Robot status: Finished")

def main():
    rospy.init_node('sensor_subscriber')

    # Subscribe to LiDAR sensor
    rospy.Subscriber('/scan', LaserScan, lidar_callback)

    # Subscribe to left and right cameras
    rospy.Subscriber('/left_camera/image_raw', Image, left_camera_callback)
    rospy.Subscriber('/right_camera/image_raw', Image, right_camera_callback)

    # Subscribe to ZED camera topics
    rospy.Subscriber('/zed2/left/image_rect_color', Image, zed_camera_callback)
    rospy.Subscriber('/zed2/right/image_rect_color', Image, zed_camera_callback)

    # Subscribe to GPS sensor
    rospy.Subscriber('/gps', NavSatFix, gps_callback)

    rospy.Subscriber('/parc_robot/robot_status', String, robot_status_callback)

    # Spin ROS
    rospy.spin()

    print("Detected objects:")
    for obj in detected_list:
        if len(obj) > 2:  # Check if geolocation information is available
            print("Coordinates:", obj[:2])
            print("Geolocation:", obj[2])
        else:
            print("Coordinates:", obj[:2])
        print(len(detected_list))

if __name__ == '__main__':
    main()
