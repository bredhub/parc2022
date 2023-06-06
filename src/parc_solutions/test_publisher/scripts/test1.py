#!/usr/bin/env python3
## Install the geographiclib 2.0 module for this code to work.
## To install geographiclib 2.0, copy the line below to your terminal.
## pip install geographiclib
## Any of the PARC competition task must be running for this code to work.

import math

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from parc_robot.gps2cartesian import gps_to_cartesian
from sensor_msgs.msg import (CameraInfo, Image, Imu, LaserScan, NavSatFix,
                             PointCloud2)
from tf.transformations import euler_from_quaternion

# Register the coordinates of the pegs
peg_01 = rospy.get_param('peg_01')
peg_02 = rospy.get_param('peg_02')
peg_03 = rospy.get_param('peg_03')
peg_04 = rospy.get_param('peg_04')
peg_05 = rospy.get_param('peg_05')
peg_06 = rospy.get_param('peg_06')
peg_07 = rospy.get_param('peg_07')
peg_08 = rospy.get_param('peg_08')

peg_01_lat, peg_01_lon = peg_01['latitude'], peg_01['longitude']
peg_02_lat, peg_02_lon = peg_02['latitude'], peg_02['longitude']
peg_03_lat, peg_03_lon = peg_03['latitude'], peg_03['longitude']
peg_04_lat, peg_04_lon = peg_04['latitude'], peg_04['longitude']
peg_05_lat, peg_05_lon = peg_05['latitude'], peg_05['longitude']
peg_06_lat, peg_06_lon = peg_06['latitude'], peg_06['longitude']
peg_07_lat, peg_07_lon = peg_07['latitude'], peg_07['longitude']
peg_08_lat, peg_08_lon = peg_08['latitude'], peg_08['longitude']



def mean_gps(x1,y1, x2, y2):
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
    
def mean_average(pointA, pointB):
    x1, y1 = pointA
    x2, y2 = pointB
    mid_x = (x1+x2)/2
    mid_y = (y1 + y2)/ 2
    return [mid_x, mid_y]
  
point1_gps = mean_gps(peg_01_lat, peg_01_lon, peg_04_lat, peg_04_lon)
point2_gps = mean_gps(peg_02_lat, peg_02_lon, peg_03_lat, peg_03_lon)
point3_gps = mean_gps(peg_03_lat, peg_03_lon,peg_06_lat, peg_06_lon)
point4_gps = mean_gps(peg_04_lat, peg_04_lon,peg_05_lat, peg_05_lon)
point5_gps = mean_gps(peg_05_lat, peg_05_lon,peg_08_lat, peg_08_lon)
point6_gps = mean_gps(peg_07_lat, peg_07_lon,peg_06_lat, peg_06_lon)
#gps to cartesian
point1 = mean_average(gps_to_cartesian(peg_01_lat, peg_01_lon), gps_to_cartesian(peg_04_lat, peg_04_lon))
point2 = mean_average(gps_to_cartesian(peg_02_lat, peg_02_lon), gps_to_cartesian(peg_03_lat, peg_03_lon))
point3 = mean_average(gps_to_cartesian(peg_03_lat, peg_03_lon), gps_to_cartesian(peg_06_lat, peg_06_lon))
point4 = mean_average(gps_to_cartesian(peg_04_lat, peg_04_lon), gps_to_cartesian(peg_05_lat, peg_05_lon))
point5 = mean_average(gps_to_cartesian(peg_05_lat, peg_05_lon), gps_to_cartesian(peg_08_lat, peg_08_lon))
point6 = mean_average(gps_to_cartesian(peg_07_lat, peg_07_lon), gps_to_cartesian(peg_06_lat, peg_06_lon))
goal_location_lat = rospy.get_param('goal_latitude')
goal_location_log = rospy.get_param('goal_longitude')
goal_location = gps_to_cartesian(goal_location_lat, goal_location_log)
all_points = [point1, point2, point3, point4, point4, point5, goal_location]
prev_index_target = 0
current_index_target = 1
current_location = all_points[current_index_target]
prev_location = all_points[prev_index_target]
main_goal_met = False
desired_angular_vel = 0.0
current_orientation = None


def odom():
    scan_data = rospy.wait_for_message('odom', Odometry)
    return scan_data


def gps():
    gps_msg = rospy.wait_for_message('/gps/fix', NavSatFix)
    current_lat = gps_msg.latitude
    current_lon = gps_msg.longitude
    x, y = gps_to_cartesian(current_lat, current_lon)
    return [x, y]

def euler_to_rotation_matrix(roll, pitch, yaw):
    # Convert Euler angles to rotation matrix
    rotation_matrix = np.zeros((3, 3))
    
    # Roll, pitch, and yaw are in radians
    cos_roll = np.cos(roll)
    sin_roll = np.sin(roll)
    cos_pitch = np.cos(pitch)
    sin_pitch = np.sin(pitch)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    
    rotation_matrix[0, 0] = cos_yaw * cos_pitch
    rotation_matrix[0, 1] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll
    rotation_matrix[0, 2] = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll
    rotation_matrix[1, 0] = sin_yaw * cos_pitch
    rotation_matrix[1, 1] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll
    rotation_matrix[1, 2] = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll
    rotation_matrix[2, 0] = -sin_pitch
    rotation_matrix[2, 1] = cos_pitch * sin_roll
    rotation_matrix[2, 2] = cos_pitch * cos_roll
    
    return rotation_matrix

def calculate_rotation_matrix(camera_orientation_quaternion):
    # Convert quaternion to Euler angles
    camera_orientation_euler = euler_from_quaternion(camera_orientation_quaternion)
    roll, pitch, yaw = camera_orientation_euler
    
    # Calculate rotation matrix from Euler angles
    rotation_matrix = euler_to_rotation_matrix(roll, pitch, yaw)
    
    return rotation_matrix

def convert_to_world_frame(blob_x, blob_y, image_width, image_height, focal_length, camera_position, camera_orientation_quaternion):
    # Convert blob coordinates to normalized image coordinates
    normalized_x = (blob_x - (image_width / 2)) / (image_width / 2)
    normalized_y = (blob_y - (image_height / 2)) / (image_height / 2)
    
    # Convert to camera frame coordinates
    camera_frame_x = normalized_x * focal_length
    camera_frame_y = normalized_y * focal_length
    camera_frame_z = focal_length
    
    # Rotate camera frame coordinates to align with the world frame
    rotation_matrix = calculate_rotation_matrix(camera_orientation_quaternion)
    camera_frame_coordinates = np.array([camera_frame_x, camera_frame_y, camera_frame_z])
    world_frame_coordinates = rotation_matrix @ camera_frame_coordinates
    
    # Translate to camera position in the world frame
    world_frame_coordinates += camera_position
    
    return world_frame_coordinates




def estimate_distance(cv_image, robot_position, image_width, image_height, camera_orientation_quaternion):
    global keypoints
    focal_length = 288
    # Convert the image to grayscale
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # Apply a threshold to convert the grayscale image to binary
    COLOR_MIN = (39, 106, 124)
    COLOR_MAX = (77, 237, 189)
    thresh_img = cv2.inRange(gray_image, COLOR_MIN, COLOR_MAX)
    
    params = cv2.SimpleBlobDetector_Params()
    # Set parameters for blob detection (change as needed)
    params.minThreshold = 10
    params.maxThreshold = 200
    params.filterByArea = True
    params.minArea = 100
    params.filterByCircularity = False
    params.filterByConvexity = False
    params.filterByInertia = False

    # Create blob detector
    detector = cv2.SimpleBlobDetector_create(params)
    
    # Detect blobs
    keypoints = detector.detect(thresh_img)
    
    image_with_blobs = cv2.drawKeypoints(thresh_img, keypoints, np.array([]), (0, 0, 255),
                                             cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # Display the image with blobs (for visualization purposes)
    cv2.imshow("Obstacle Detection", image_with_blobs)
    cv2.waitKey(1)
    # print(keypoints)
    obstacle_detected = len(keypoints) > 0
    # keypoints = keypoints
    robot_x = robot_position[0]
    
    robot_y = robot_position[1]
    obstacle_distances = []
    for keypoint in keypoints:
        blob_x, blob_y = keypoint.pt
        world_frame_coordinates = convert_to_world_frame(blob_x, blob_y, image_width, image_height, focal_length,
                                                         robot_position, camera_orientation_quaternion)
        
        distance_to_robot = math.sqrt((world_frame_coordinates[0] - robot_x) ** 2 + (world_frame_coordinates[1] - robot_y) ** 2)
        obstacle_distances.append(distance_to_robot)
    print(obstacle_distances)
    # print("obstacel")
    if obstacle_detected:
        min_distance = min(obstacle_distances)
        # Check if the minimum distance is below the obstacle distance threshold
        if min_distance < 200:
            # Perform obstacle avoidance actions
            # Example: Stop the robot, change direction, etc.
            print("Obstacle detected. Taking avoidance action.")
            return True
    
    # return False

def analyse_image(scan_data, robot_position, odom_position):
    # if camera_info is None:
    #     rospy.logwarn('Camera info not available yet.')
    #     return False
    try:
        orientation = odom_position.pose.pose.orientation
        
        # Convert the orientation to Euler angles
        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        quaternion = euler_from_quaternion.quaternion_from_euler(roll, pitch, yaw)
        image_width = scan_data.width
        image_height = scan_data.height
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(scan_data, desired_encoding='bgr8')
        turn = False
        
        # Perform image processing and distance estimation
        obstacle_detected = estimate_distance(cv_image, robot_position, image_width, image_height, quaternion)
        return obstacle_detected
    except Exception as e:
        rospy.logerr(f"Error processing image: {str(e)}")
        return False

def camera():
    scan_data = rospy.wait_for_message('/right_camera/image_raw', Image)
    return scan_data

def act(robot_vel_publisher,  robot_position):
    global desired_angular_vel
    robot_vel = Twist()
    fwd_vel = 0.2
    
    if True:
        #continue moving
        robot_vel.linear.x = fwd_vel
        robot_vel.angular.z = 0.0
        msg = "Robot continue moving \n"
        robot_vel_publisher.publish(robot_vel)
     
    return msg
    

def main():
    rospy.init_node('collision_avoidance_node')
    
    hz = 10
    rate = rospy.Rate(hz)

    robot_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    
    while not rospy.is_shutdown():
        
        camera_scan = camera()
        
        #get position 
        robot_position = gps()
        
        #where the robot is facing
        position_robot = odom()
        
        #front camera
        front_call = analyse_image(camera_scan, robot_position, position_robot)
        
        
        message = act(robot_vel_publisher,  robot_position)
    
        
        print(f'At time []: {message}!')

        rate.sleep()


if __name__ == '__main__':
    main()